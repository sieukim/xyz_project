import 'dart:async';
import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:flutter_tts/flutter_tts.dart';
import 'package:speech_to_text/speech_to_text.dart';
import 'package:http/http.dart' as http;

import 'package:smart_fuel/widgets/realsense_view.dart';
import 'package:smart_fuel/services/llm_service.dart';
import 'package:smart_fuel/widgets/webcam_view.dart';

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
    _initTtsAndSpeak();
    _initStt();
    _extractIpAndStartPolling();
  }

  Future<void> _initTtsAndSpeak() async {
    await _flutterTts.setLanguage('ko-KR');
    await _flutterTts.setSpeechRate(1.0);
    // speak 함수가 즉시 반환되도록 awaitSpeakCompletion 설정을 false로 변경합니다.
    await _flutterTts.awaitSpeakCompletion(false);

    // 음성 출력 시작/종료 시 _isSpeaking 상태를 업데이트하여 UI를 제어합니다.
    _flutterTts.setStartHandler(() {
      if (mounted) setState(() => _isSpeaking = true);
    });
    _flutterTts.setCompletionHandler(() {
      if (mounted) setState(() => _isSpeaking = false);
    });
    _flutterTts.setErrorHandler((msg) {
      if (mounted) setState(() => _isSpeaking = false);
    });
    await _flutterTts.speak("주유중입니다.");
  }

  Future<void> _initStt() async {
    await _speechToText.initialize(
      onStatus: (status) {}, // 상태 변경을 여기서 직접 처리하지 않음
    );
  }

  void _toggleListening() {
    if (!_speechToText.isAvailable || _completed || _isFinishingSoon) return;
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
      listenFor: const Duration(seconds: 5), // 5초 후 자동으로 듣기 종료
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

    final prompt = """
        사용자의 질문 의도를 다음 중 하나로 분류하고 JSON 형식으로 반환해줘.
        사용 가능한 의도: 'progress_query', 'time_query', 'hungry_query', 'wipe_query', 'thirsty_query', 'news_query', 'weather_query', 'joke_query', 'music_query', 'book_query', 'movie_query', 'restaurant_query', 'travel_query', 'health_query', 'tech_query', 'finance_query', 'etc'.
    
        - 'progress_query': 주유 진행률(%)을 묻는 질문. (예: "얼마나 됐어?", "진행률 알려줘")
        - 'time_query': 남은 시간을 묻는 질문. (예: "몇 초 남았어?", "언제 끝나?")
        - 'hungry_query': 배고픔과 관련된 표현. (예: "배고파", "출출한데", "뭐 먹을 거 없어?")
        - 'wipe_query': 무언가 닦을 것이 필요하다는 표현. (예: "뭐 닦아야 하는데", "휴지 좀")
        - 'thirsty_query': 목마름과 관련된 표현. (예: "목말라", "마실 것 좀 줘")
        - 'news_query', 'weather_query', 'joke_query', 'music_query', 'book_query', 'movie_query', 'restaurant_query', 'travel_query', 'health_query', 'tech_query', 'finance_query': 일반적인 정보성 질문.
        - 'etc': 위 분류에 해당하지 않는 모든 경우.
    
        결과 예시: {"intent": "time_query"}
    
        사용자 질문: "$command"
        """;

 try {
      final result = await LlmService.generateContent(prompt);
      final intent = result['intent'] as String? ?? 'etc';
      String responseText;

      switch (intent) {
        case 'progress_query':
          final remainingPercent = 100 - _progress;
          responseText = "현재 $_progress% 완료, 약 $remainingPercent% 남았습니다.";
          break;
        case 'time_query':
          const totalDuration = 60.0; // 총 주유 시간 (초)
          final remainingSeconds = (totalDuration * (100 - _progress) / 100).round();
          responseText = "약 $remainingSeconds초 남았습니다.";
          break;
        case 'hungry_query':
          responseText = "간식을 가져다드리겠습니다. (약 30초 소요)";
          break;
        case 'wipe_query':
          responseText = "휴지를 가져다드리겠습니다. (약 10초 소요)";
          break;
        case 'thirsty_query':
          responseText = "물을 가져다드리겠습니다. (약 20초 소요)";
          break;
        case 'news_query':
        case 'weather_query':
        case 'joke_query':
        case 'music_query':
        case 'book_query':
        case 'movie_query':
        case 'restaurant_query':
        case 'travel_query':
        case 'health_query':
        case 'tech_query':
        case 'finance_query':
        default:
          // LLM을 사용하여 자연스러운 답변 생성
          final responsePrompt = """
          당신은 주유 중인 운전자를 돕는 친절한 AI 비서입니다.
          사용자의 질문에 대해 간결하고 친절한 한 문장으로 답변해주세요.
          
          사용자 질문: "$command"
          """;
          responseText = await LlmService.generateTextOnly(responsePrompt);
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

  /// 음성 인식 및 분석이 진행 중인지 여부를 반환합니다.
  bool get _isVoiceProcessing => _isListening || _voiceCommandStatusText == '분석 중...' || _isSpeaking;

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
          
          const totalDuration = 60.0; // 총 주유 시간 (초)
          final remainingSeconds = (totalDuration * (100 - p) / 100).round();

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
      Navigator.popUntil(context, (route) => route.isFirst);
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
                                RealSenseViewWidget(serverIp: _serverIp!),
                                VideoViewWidget(serverIp: _serverIp!),
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
                            tabs: const [
                              Tab(text: '로봇 뷰'),
                              Tab(text: '차량 뷰'),
                            ],
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
                        _isVoiceProcessing || _isFinishingSoon ? Icons.mic_off : Icons.mic,
                        color: _isVoiceProcessing || _isFinishingSoon ? Colors.grey : darkGrayText,
                      ),
                      onPressed: _isVoiceProcessing || _isFinishingSoon ? null : _toggleListening,
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
