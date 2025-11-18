import 'dart:async';
import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:flutter_tts/flutter_tts.dart';
import 'package:speech_to_text/speech_to_text.dart';
import 'package:http/http.dart' as http;

import 'package:smart_fuel/config/app_config.dart';
import 'package:smart_fuel/widgets/realsense_view.dart';
import 'package:smart_fuel/services/llm_service.dart';
import 'package:smart_fuel/widgets/webcam_view.dart';
import 'package:smart_fuel/screens/fuel_selection_screen.dart';

class FuelProgressScreen extends StatefulWidget {
  final String orderId;
  final String rosBaseUrl;

  const FuelProgressScreen(
      {Key? key, required this.orderId, required this.rosBaseUrl})
      : super(key: key);

  @override
  State<FuelProgressScreen> createState() => _FuelProgressScreenState();
}

class _FuelProgressScreenState extends State<FuelProgressScreen>
    with SingleTickerProviderStateMixin {
  final FlutterTts _flutterTts = FlutterTts();
  final SpeechToText _speechToText = SpeechToText();
  bool _isListening = false;
  String _voiceCommandStatusText = '주유 중 궁금한 점을 말씀해주세요.';
  bool _isSpeaking = false;

  bool _voiceFeatureActive = true; // 음성 기능 활성화 여부
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
    _tabController = TabController(length: 2, vsync: this);
    _initializeScreen();
  }

  Future<void> _initializeScreen() async {
    await _initTts();
    await _initStt();
    _extractIpAndStartPolling();
    // 화면 빌드 후 초기 대화 시작
    WidgetsBinding.instance.addPostFrameCallback((_) => _startInitialConversation());
  }

  Future<void> _initTts() async {
    await _flutterTts.setLanguage('ko-KR');
    await _flutterTts.setSpeechRate(1.0);
    await _flutterTts.awaitSpeakCompletion(true);
  }

  Future<void> _startInitialConversation() async {
    if (!mounted) return;
    await _flutterTts.speak("주유중입니다.");
    await _flutterTts.speak("도움이 필요하신게 있으신가요?");
    if (mounted) {
      setState(() => _voiceCommandStatusText = '도움이 필요하신게 있으신가요?');
      _startListening();
    }
  }

  Future<void> _initStt() async {
    await _speechToText.initialize(
      onStatus: (status) {}, // 상태 변경을 여기서 직접 처리하지 않음
    );
  }

  void _toggleListening() {
    if (!_speechToText.isAvailable || _completed || _isFinishingSoon || !_voiceFeatureActive) return;
    if (_isListening) {
      _stopListening();
    } else {
      _startListening();
    }
  }

  void _startListening() {
    if (!_speechToText.isAvailable) return;
    setState(() {
      _isListening = true;
      _voiceCommandStatusText = '듣는 중...';
    });
    _speechToText.listen(
      onResult: (result) {
        if (result.finalResult) {
          _processVoiceCommand(result.recognizedWords);
        }
      },
      localeId: 'ko_KR',
      listenFor: const Duration(seconds: 2), // 2초로 수정
    );
  }

  void _stopListening() {
    _speechToText.stop();
    if (mounted) setState(() => _isListening = false);
  }

  Future<void> _processVoiceCommand(String command) async {
    if (command.isEmpty) return;

    setState(() {
      _voiceCommandStatusText = '분석 중...';
      _isListening = false;
    });

    final totalDuration = AppConfig.totalFuelingSeconds;
    final remainingSeconds = (totalDuration * (100 - _progress) / 100.0).round();

    final prompt = """
    당신은 주유 중인 운전자를 돕는 친절한 AI 비서입니다. 사용자의 질문에 대해 JSON 형식으로 답변을 생성해주세요.

    ### 현재 주유 상태 (내부 정보):
    - 진행률: $_progress%
    - 남은 시간: $remainingSeconds초

    ### 지침:
    1.  사용자의 질문 의도를 파악하세요.
    2.  '진행률'이나 '남은 시간'에 대한 질문이라면, 위 내부 정보를 활용하여 자연스러운 한 문장으로 답변을 생성하세요.
    3.  사용자가 도움이 필요 없다고 말하면(예: "아니", "없어", "괜찮아"), 'intent' 필드에 'no_help_needed'를 포함시키세요.
    4.  그 외 모든 질문(날씨, 농담, 일반 상식 등)에 대해서도 사용자를 즐겁게 할 수 있는 간결하고 친절한 답변을 생성하세요.
    5.  생성된 답변은 'response' 필드에 담아 JSON으로 반환하세요.

    ### 예시:
    - 사용자 질문: "얼마나 됐어?"
      -> {"response": "네, 현재 $_progress% 진행되었습니다. 거의 다 되어가네요!"}
    - 사용자 질문: "아니 괜찮아"
      -> {"intent": "no_help_needed", "response": "네, 알겠습니다. 편하게 기다려주세요."}
    - 사용자 질문: "오늘 날씨 어때?"
      -> {"response": "오늘 날씨는 화창해서 주유 후에 드라이브하기 딱 좋은 날씨입니다!"}
    - 사용자 질문: "재미있는 얘기 해줘."
      -> {"response": "기름을 너무 많이 먹는 공룡 이름은 뭘까요? 바로 '기름이모자우루스'입니다!"}

    ### 사용자 질문:
    "$command"
    """;

 try {
      final result = await LlmService.generateContent(prompt);
      final responseText = result['response'] as String?;
      final intent = result['intent'] as String?;

      if (intent == 'no_help_needed') {
        setState(() => _voiceFeatureActive = false);
      }
      
      if (responseText == null || responseText.isEmpty) {
        throw Exception('LLM이 유효한 답변을 생성하지 못했습니다.');
      }

      // 1. 화면의 텍스트를 먼저 업데이트합니다.
      if (mounted) {
        setState(() {
          _voiceCommandStatusText = responseText;
        });
      }
      // 2. UI가 실제로 렌더링된 다음 TTS 실행
      await _flutterTts.speak(responseText);
    } catch (e) {
      debugPrint('LLM 의도 분석 오류: $e');
      // API 오류 발생 시 사용자에게 피드백 제공
      const String errorMessage = 'AI 서버가 응답하지 않습니다. 잠시 후 다시 시도해주세요.';
      await _flutterTts.speak(errorMessage);
      if (mounted) {
        setState(() {
          _voiceCommandStatusText = errorMessage;
        });
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
    _countdownTimer?.cancel();
    _flutterTts.stop();
    super.dispose();
  }

  /// 음성 인식 및 분석이 진행 중인지 여부를 반환합니다.
  bool get _isVoiceProcessing => _isListening || _voiceCommandStatusText == '분석 중...' || _isSpeaking;

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

    await _flutterTts.speak("주유를 완료했습니다. 안녕히 가세요.");

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
                    _completed ? '주유 완료!' : '주유 중입니다...',
                    style: const TextStyle(fontSize: 22, fontWeight: FontWeight.bold, color: darkGrayText),
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
              Container(
                padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
                decoration: BoxDecoration(
                  color: lightGrayBg,
                  borderRadius: BorderRadius.circular(12),
                ),
                child: Row(
                  children: [
                    IconButton(
                      icon: Icon(
                        !_voiceFeatureActive || _isVoiceProcessing || _isFinishingSoon ? Icons.mic_off : Icons.mic,
                        color: !_voiceFeatureActive || _isVoiceProcessing || _isFinishingSoon ? Colors.grey : darkGrayText,
                      ),
                      onPressed: !_voiceFeatureActive || _isVoiceProcessing || _isFinishingSoon ? null : _toggleListening,
                      tooltip: '궁금한 점을 질문하세요',
                    ),
                    const SizedBox(width: 8),
                    Expanded(
                      child: Text(
                        _voiceCommandStatusText,
                        style: TextStyle(fontSize: 16, color: _isFinishingSoon ? Colors.grey : lightGrayText),
                      ),
                    ),
                  ],
                ),
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
