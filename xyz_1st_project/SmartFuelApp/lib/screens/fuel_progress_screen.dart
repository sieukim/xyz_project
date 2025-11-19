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
import 'package:smart_fuel/theme/app_theme.dart';

/// 주유 진행 상태를 보여주는 화면
class FuelProgressScreen extends StatefulWidget {
  final String orderId;
  final String rosBaseUrl;

  const FuelProgressScreen(
      {Key? key, required this.orderId, required this.rosBaseUrl})
      : super(key: key);

  @override
  State<FuelProgressScreen> createState() => _FuelProgressScreenStateWrapper();
}

/// VoiceInteractionService를 하위 위젯에 제공하기 위한 래퍼
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

/// 주유 진행 화면의 실제 UI와 상태를 포함하는 위젯
class _FuelProgressScreenContent extends StatefulWidget {
  final String orderId;
  final String rosBaseUrl;

  const _FuelProgressScreenContent({required this.orderId, required this.rosBaseUrl});

  @override
  State<_FuelProgressScreenContent> createState() => _FuelProgressScreenState();
}

/// 주유 진행 화면의 상태 관리 로직 (서버 폴링, 음성 대화, UI 업데이트)
class _FuelProgressScreenState extends State<_FuelProgressScreenContent>
    with SingleTickerProviderStateMixin {
  late VoiceInteractionService _voiceService;
  Timer? _timer;
  String _status = '대기 중';
  int _progress = 0;
  bool _completed = false;
  int _countdown = 5;
  bool _isFinishingSoon = false; 
  Timer? _countdownTimer;
  TabController? _tabController;
  String? _serverIp;

  /// 위젯 초기화
  @override
  void initState() {
    super.initState();
    _voiceService = Provider.of<VoiceInteractionService>(context, listen: false);
    _tabController = TabController(length: 2, vsync: this);
    _initializeScreen();
  }

  /// 화면에 필요한 서비스 초기화 및 폴링 시작
  Future<void> _initializeScreen() async {
    _voiceService.onResult = _processVoiceCommand;
    _extractIpAndStartPolling();
    WidgetsBinding.instance.addPostFrameCallback((_) => _startInitialConversation());
  }

  /// 초기 음성 안내 시작
  Future<void> _startInitialConversation() async {
    if (!mounted) return;
    await _voiceService.speak("주유중입니다.");
    _voiceService.speakAndListen("도움이 필요하신게 있으신가요?");
  }

  /// 사용자 음성 명령 처리
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

  /// 대화 분석 결과에 따른 액션 수행
  void _handleConversationAction(ConversationAction action) {
    if (!mounted) return;

    if (action.stateUpdate['deactivate_voice'] == true) {
      _voiceService.stopAndDeactivate();
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

  /// 서버 URL에서 IP를 추출하고 상태 폴링 시작
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

  /// 위젯 종료 시 타이머 정리
  @override
  void dispose() {
    _timer?.cancel();
    _tabController?.dispose();
    super.dispose();
  }

  /// 서버로부터 주유 상태 주기적으로 조회
  Future<void> _fetchStatus() async {
    if (_serverIp == null) {
      debugPrint("Server IP not yet extracted, skipping poll.");
      return;
    }

    final statusUrl = Uri.parse('http://$_serverIp:${AppConfig.rosApiPort}/status/${widget.orderId}');

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
    } catch (e) { /* 네트워크 에러는 무시 */ }
  }

  /// 주유 완료 시 처리 로직
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

  /// 홈 화면(주유 설정)으로 이동
  void _navigateToHome() {
    if (mounted) {
      _countdownTimer?.cancel();
      Navigator.pushAndRemoveUntil(
        context,
        MaterialPageRoute(builder: (context) => const FuelSelectionScreen()),
        (Route<dynamic> route) => false, 
      );
    }
  }

  /// 화면 UI 구성
  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);

    return Scaffold(
      appBar: AppBar(
        title: const Text('주유 진행 상황'),
        automaticallyImplyLeading: false,
      ),
      body: Padding(
        padding: const EdgeInsets.all(20),
        child: Column(
          children: [
            Expanded(
              flex: 5,
              child: _serverIp == null
                  ? Center(child: CircularProgressIndicator(strokeWidth: 2, color: theme.primaryColor))
                  : Card(
                      clipBehavior: Clip.antiAlias,
                      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
                      elevation: 0,
                      color: AppColors.surface,
                      child: Column(
                        children: [
                          Expanded(
                            child: TabBarView(
                              controller: _tabController,
                              children: [
                                RealSenseViewWidget(
                                  serverIp: AppConfig.rosIp,
                                  streamPort: AppConfig.realsenseStreamerPort.toString(),
                                ),
                                VideoViewWidget(
                                  serverIp: AppConfig.rosIp,
                                  streamPort: AppConfig.webcamStreamerPort.toString(),
                                ),
                              ],
                            ),
                          ),
                          TabBar(
                            controller: _tabController,
                            labelColor: AppColors.textPrimary,
                            unselectedLabelColor: AppColors.textSecondary,
                            indicatorColor: theme.primaryColor,
                            indicatorWeight: 3.0,
                            dividerColor: Colors.transparent,
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
                color: AppColors.surface,
                borderRadius: BorderRadius.circular(16),
              ),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    _completed ? '주유가 완료되었습니다!' : '주유 중입니다...',
                    style: theme.textTheme.headlineSmall,
                  ),
                  const SizedBox(height: 8),
                  Text(
                    '주문 ID: ${widget.orderId}', 
                    style: theme.textTheme.bodySmall,
                  ),
                  const SizedBox(height: 20),
                  LinearProgressIndicator(
                    value: _progress / 100.0,
                    minHeight: 10,
                    backgroundColor: Colors.grey[300],
                    color: theme.primaryColor,
                  ),
                  const SizedBox(height: 8),
                  Align(
                    alignment: Alignment.centerRight,
                    child: Text(
                      '$_progress%',
                      style: theme.textTheme.bodyMedium?.copyWith(
                          fontWeight: FontWeight.bold, color: AppColors.textPrimary),
                    ),
                  ),
                ],
              ),
            ),
            const SizedBox(height: 24),
            if (!_completed)
              VoiceCommandBar(
                initialText: '주유 중 궁금한 점을 말씀해주세요.',
                backgroundColor: AppColors.surface,
                textColor: AppColors.textPrimary,
              ),
            if (_completed) const Spacer(),
          ],
        ),
      ),
      bottomNavigationBar: Padding(
        padding: const EdgeInsets.all(20.0),
        child: ElevatedButton(
          onPressed: _completed ? _navigateToHome : null,
          style: theme.elevatedButtonTheme.style,
          child: Text(
            _completed ? '$_countdown초 후 홈으로 이동' : '주유 중입니다',
          ),
        ),
      ),
    );
  }
}
