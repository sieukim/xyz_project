import 'dart:async';
import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'package:provider/provider.dart';
import 'package:smart_fuel/services/conversation_manager_service.dart';
import 'package:smart_fuel/services/voice_interaction_service.dart';
import 'package:smart_fuel/config/app_config.dart';
import 'package:smart_fuel/widgets/realsense_view.dart';
import 'package:smart_fuel/services/llm_service.dart';
import 'package:smart_fuel/widgets/voice_command_bar.dart';
import 'package:smart_fuel/widgets/webcam_view.dart';
import 'package:smart_fuel/screens/fuel_selection_screen.dart';

class FuelProgressScreen extends StatefulWidget {
  final String orderId;
  final String rosBaseUrl;

  const FuelProgressScreen(
      {Key? key, required this.orderId, required this.rosBaseUrl})
      : super(key: key);

  @override
  State<FuelProgressScreen> createState() => _FuelProgressScreenStateWrapper();
}

/// ChangeNotifierProvider를 사용하기 위한 Wrapper 클래스
class _FuelProgressScreenStateWrapper extends State<FuelProgressScreen> {
  @override
  Widget build(BuildContext context) {
    return ChangeNotifierProvider(
      create: (_) => VoiceInteractionService(),
      child: _FuelProgressScreenContent(
        orderId: widget.orderId,
        rosBaseUrl: widget.rosBaseUrl,
      ),
    );
  }
}

class _FuelProgressScreenContent extends StatefulWidget {
  final String orderId;
  final String rosBaseUrl;

  const _FuelProgressScreenContent({required this.orderId, required this.rosBaseUrl});

  @override
  State<_FuelProgressScreenContent> createState() => _FuelProgressScreenState();
}

class _FuelProgressScreenState extends State<_FuelProgressScreenContent>
    with SingleTickerProviderStateMixin {
  late VoiceInteractionService _voiceService;
  Timer? _timer;
  String _status = '대기 중';
  int _progress = 0;
  bool _completed = false;
  int _countdown = 5;
  bool _isFinishingSoon = false; // 주유 완료 임박 상태
  Timer? _countdownTimer;

  TabController? _tabController;
  String? _serverIp;

  @override
  void initState() {
    super.initState();
    // 탭 컨트롤러 초기화
    _voiceService = Provider.of<VoiceInteractionService>(context, listen: false);
    _tabController = TabController(length: 2, vsync: this);
    _initializeScreen();
  }

  Future<void> _initializeScreen() async {
    _voiceService.onResult = _processVoiceCommand;
    _extractIpAndStartPolling();
    // 화면 빌드 후 초기 대화 시작
    WidgetsBinding.instance.addPostFrameCallback((_) => _startInitialConversation());
  }

  Future<void> _startInitialConversation() async {
    if (!mounted) return;
    await _voiceService.speak("주유중입니다.");
    _voiceService.speakAndListen("도움이 필요하신게 있으신가요?");
  }

  /// TTS로 문장을 말하고, 끝나면 바로 음성 인식을 시작하는 헬퍼 함수

  Future<void> _processVoiceCommand(String command) async {
    if (command.isEmpty) return;

    final totalDuration = AppConfig.totalFuelingSeconds;
    final remainingSeconds = (totalDuration * (100 - _progress) / 100.0).round();

    final manager = ConversationManagerService();
    final action = await manager.processVoiceCommand(
      command: command,
      screen: ConversationScreen.fuelProgress,
      context: {
        'progress': _progress,
        'remainingSeconds': remainingSeconds,
      },
    );

    _handleConversationAction(action);
  }

  void _handleConversationAction(ConversationAction action) {
    if (!mounted) return;

    if (action.stateUpdate['deactivate_voice'] == true) {
      _voiceService.deactivateFeature(action.speakText ?? "네, 알겠습니다.");
      return;
    }

    if (action.speakText != null && _voiceService.isFeatureActive) {
      if (action.shouldListenNext) {
        _voiceService.speakAndListen(action.speakText!);
      } else {
        _voiceService.speak(action.speakText!);
      }
    }
  }

  void _extractIpAndStartPolling() {
    try {
      final uri = Uri.parse(widget.rosBaseUrl);
      setState(() {
        _serverIp = uri.host;
      });

      _fetchStatus();
      _timer = Timer.periodic(const Duration(seconds: 2), (_) => _fetchStatus());
    } catch (e) {
      debugPrint("Invalid rosBaseUrl: $e");
      setState(() {
        _status = "서버 URL 오류";
      });
    }
  }

  @override
  void dispose() {
    _timer?.cancel();
    _tabController?.dispose();
    super.dispose();
  }

  Future<void> _fetchStatus() async {
    if (_serverIp == null) {
      debugPrint("Server IP not yet extracted, skipping poll.");
      return;
    }

    final statusUrl = Uri.parse('http://$_serverIp:8000/status/${widget.orderId}');

    try {
      final res = await http.get(statusUrl).timeout(const Duration(seconds: 6));
      if (res.statusCode == 200) {
        final body = jsonDecode(res.body);
        if (body is Map) {
          final s = (body['status'] ?? '').toString();
          final p = (int.tryParse(body['progress']?.toString() ?? '0') ?? 0).clamp(0, 100);
          
          final totalDuration = AppConfig.totalFuelingSeconds;
          final remainingSeconds = (totalDuration * (100 - p) / 100.0).round();

          setState(() {
            _status = s.isNotEmpty ? s : _status;
            _progress = p;
            _isFinishingSoon = remainingSeconds <= 20;
          });
          if (s == 'completed' || s == 'done' || s == 'finished') {
            _onCompleted();
          }
        }
      }
    } catch (e) {
      // 네트워크 에러는 무시
    }
  }

  Future<void> _onCompleted() async {
    if (_completed) return;
    _timer?.cancel();

    if (!mounted) return;
    setState(() {
      _completed = true;
      _status = '주유 완료';
      _progress = 100;
    });

    await _voiceService.speak("주유를 완료했습니다. 안녕히 가세요.");

    _countdownTimer = Timer.periodic(const Duration(seconds: 1), (timer) {
      if (_countdown > 1) {
        setState(() {
          _countdown--;
        });
      } else {
        timer.cancel();
        _navigateToHome();
      }
    });
  }

  void _navigateToHome() {
    if (mounted) {
      _countdownTimer?.cancel();
      // 현재 화면이 스택의 첫 번째 화면이므로 popUntil이 동작하지 않습니다.
      // 모든 이전 경로를 지우고 FuelSelectionScreen으로 이동합니다.
      Navigator.pushAndRemoveUntil(
        context,
        MaterialPageRoute(builder: (context) => const FuelSelectionScreen()),
        (Route<dynamic> route) => false, // 모든 이전 라우트를 제거합니다.
      );
    }
  }

  @override
  Widget build(BuildContext context) {
    const tossBlue = Color(0xFF3182F7);
    const darkGrayText = Color(0xFF333D4B);
    const lightGrayText = Color(0xFF6B7684);
    const lightGrayBg = Color(0xFFF2F4F6);
    const white = Colors.white;

    return Scaffold(
      backgroundColor: white,
      appBar: AppBar(
        title: const Text('주유 진행 상황', style: TextStyle(color: darkGrayText)),
        backgroundColor: white,
        elevation: 0,
        iconTheme: const IconThemeData(color: darkGrayText),
        automaticallyImplyLeading: false,
      ),
      body: Padding(
        padding: const EdgeInsets.all(20),
        child: Column(
          children: [
            Expanded(
              flex: 5,
              child: _serverIp == null
                  ? const Center(child: CircularProgressIndicator(strokeWidth: 2, color: tossBlue))
                  : Card(
                      clipBehavior: Clip.antiAlias,
                      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
                      elevation: 0,
                      color: lightGrayBg,
                      child: Column(
                        children: [
                          Expanded(
                            child: TabBarView(
                              controller: _tabController,
                              children: [
                                // 로봇 뷰 (RealSense)
                                RealSenseViewWidget(
                                  serverIp: AppConfig.rosIp,
                                  streamPort: AppConfig.realsenseStreamerPort.toString(),
                                ),
                                // 차량 뷰 (Webcam)
                                VideoViewWidget(
                                  serverIp: AppConfig.rosIp,
                                  streamPort: AppConfig.webcamStreamerPort.toString(),
                                ),
                              ],
                            ),
                          ),
                          TabBar(
                            controller: _tabController,
                            labelColor: darkGrayText,
                            unselectedLabelColor: lightGrayText,
                            indicatorColor: tossBlue,
                            indicatorWeight: 3.0,
                            dividerColor: Colors.transparent, // 탭 하단 구분선 제거
                            tabs: const [Tab(text: '로봇 뷰'), Tab(text: '차량 뷰')],
                          ),
                        ],
                      ),
                    ),
            ),
            const SizedBox(height: 24),
            Container(
              padding: const EdgeInsets.all(24),
              decoration: BoxDecoration(
                color: lightGrayBg,
                borderRadius: BorderRadius.circular(16),
              ),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    _completed ? '주유가 완료되었습니다!' : '주유 중입니다...',
                    style: const TextStyle(fontSize: 24, fontWeight: FontWeight.bold, color: darkGrayText),
                  ),
                  const SizedBox(height: 8),
                  Text(
                    '주문 ID: ${widget.orderId}', 
                    style: const TextStyle(fontSize: 14, color: lightGrayText),
                  ),
                  const SizedBox(height: 20),
                  LinearProgressIndicator(
                    value: _progress / 100.0,
                    minHeight: 10,
                    backgroundColor: Colors.grey[300],
                    color: tossBlue,
                  ),
                  const SizedBox(height: 8),
                  Align(
                    alignment: Alignment.centerRight,
                    child: Text(
                      '$_progress%',
                      style: const TextStyle(fontSize: 16, fontWeight: FontWeight.bold, color: darkGrayText),
                    ),
                  ),
                ],
              ),
            ),
            const SizedBox(height: 24),
            if (!_completed)
              VoiceCommandBar(
                initialText: '주유 중 궁금한 점을 말씀해주세요.',
                backgroundColor: lightGrayBg,
                textColor: darkGrayText,
              ),
            if (_completed) const Spacer(),
          ],
        ),
      ),
      bottomNavigationBar: Padding(
        padding: const EdgeInsets.all(20.0),
        child: ElevatedButton(
          onPressed: _completed ? _navigateToHome : null,
          style: ElevatedButton.styleFrom(
            backgroundColor: tossBlue,
            foregroundColor: white,
            disabledBackgroundColor: lightGrayBg,
            disabledForegroundColor: darkGrayText.withOpacity(0.38),
            padding: const EdgeInsets.symmetric(vertical: 16),
            shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
            elevation: 0,
          ),
          child: Text(
            _completed ? '$_countdown초 후 홈으로 이동' : '주유 중입니다',
            style: const TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
          ),
        ),
      ),
    );
  }
}
