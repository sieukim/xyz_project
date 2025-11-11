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

class _FuelProgressScreenState extends State<FuelProgressScreen> {
  final FlutterTts _flutterTts = FlutterTts();
  final SpeechToText _speechToText = SpeechToText();
  bool _isListening = false;
  String _voiceCommandStatusText = '주유 중 궁금한 점을 말씀해주세요.';

  Timer? _timer;
  String _status = '대기 중';
  int _progress = 0;
  bool _completed = false;
  int _countdown = 5;
  Timer? _countdownTimer;

  String? _serverIp;

  @override
  void initState() {
    super.initState();
    _initTtsAndSpeak();
    _initStt();
    _extractIpAndStartPolling();
  }

  Future<void> _initTtsAndSpeak() async {
    await _flutterTts.setLanguage('ko-KR');
    await _flutterTts.setSpeechRate(1.0);
    await _flutterTts.speak("주유중입니다.");
  }

  Future<void> _initStt() async {
    await _speechToText.initialize();
  }

  void _toggleListening() {
    if (!_speechToText.isAvailable || _completed) return;
    if (_isListening) {
      _speechToText.stop();
      setState(() => _isListening = false);
    } else {
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
  }

  Future<void> _processVoiceCommand(String command) async {
    if (command.isEmpty) return;

    setState(() {
      _voiceCommandStatusText = '분석 중...';
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
      String responseText = '';

      switch (intent) {
        case 'progress_query':
          final remainingPercent = 100 - _progress;
          responseText = "현재 $_progress% 완료, 약 $remainingPercent% 남았습니다.";
          await _flutterTts.speak(responseText);
          break;
        case 'time_query':
          const totalDuration = 20.0; // 총 주유 시간 (초)
          final remainingSeconds = (totalDuration * (100 - _progress) / 100).round();
          responseText = "약 $remainingSeconds초 남았습니다.";
          await _flutterTts.speak(responseText);
          break;
        case 'hungry_query':
          responseText = "간식을 가져다드리겠습니다. (약 30초 소요)";
          await _flutterTts.speak(responseText);
          break;
        case 'wipe_query':
          responseText = "휴지를 가져다드리겠습니다. (약 10초 소요)";
          await _flutterTts.speak(responseText);
          break;
        case 'thirsty_query':
          responseText = "물을 가져다드리겠습니다. (약 20초 소요)";
          await _flutterTts.speak(responseText);
          break;
        case 'news_query':
          responseText = "최신 뉴스를 검색해볼게요.";
          await _flutterTts.speak(responseText);
          break;
        case 'weather_query':
          responseText = "현재 위치의 날씨 정보를 알려드릴게요.";
          await _flutterTts.speak(responseText);
          break;
        case 'joke_query':
          responseText = "세상에서 가장 뜨거운 과일이 뭔지 아세요? 바로 천도복숭아입니다.";
          await _flutterTts.speak(responseText);
          break;
        case 'music_query':
          responseText = "들으실 만한 음악을 추천해 드릴게요.";
          await _flutterTts.speak(responseText);
          break;
        case 'book_query':
          responseText = "읽을 만한 책을 찾아볼게요.";
          await _flutterTts.speak(responseText);
          break;
        case 'movie_query':
          responseText = "요즘 볼만한 영화를 추천해 드릴게요.";
          await _flutterTts.speak(responseText);
          break;
        case 'restaurant_query':
          responseText = "주변 맛집을 검색해 드릴게요.";
          await _flutterTts.speak(responseText);
          break;
        case 'travel_query':
          responseText = "가실 만한 여행지를 추천해 드릴게요.";
          await _flutterTts.speak(responseText);
          break;
        case 'health_query':
        case 'tech_query':
        case 'finance_query':
          responseText = "요청하신 분야의 정보를 찾아볼게요.";
          await _flutterTts.speak(responseText);
          break;
        default:
          responseText = '요청을 이해하지 못했어요. 다시 말씀해주세요.';
          await _flutterTts.speak(responseText);
          break;
      }

      if (mounted) {
        setState(() {
          _voiceCommandStatusText = responseText;
        });
      }
    } catch (e) {
      debugPrint('LLM 의도 분석 오류: $e');
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
          final p = int.tryParse(body['progress']?.toString() ?? '') ?? 0;
          setState(() {
            _status = s.isNotEmpty ? s : _status;
            _progress = p.clamp(0, 100);
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

    await _flutterTts.speak("주유를 완료했습니다. 안녕히 가세요.");

    if (!mounted) return;
    setState(() {
      _completed = true;
      _status = '주유 완료';
      _progress = 100;
    });

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
                  : Column(
                      children: [
                        Expanded(
                          child: Card(
                            clipBehavior: Clip.antiAlias,
                            shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
                            elevation: 0,
                            color: lightGrayBg,
                            child: VideoViewWidget(serverIp: _serverIp!),
                          ),
                        ),
                        const SizedBox(height: 10),
                        Expanded(
                          child: Card(
                            shape: RoundedRectangleBorder(
                                borderRadius: BorderRadius.circular(12)),
                            clipBehavior: Clip.antiAlias,
                            elevation: 0,
                            color: lightGrayBg,
                            child: RealSenseViewWidget(serverIp: _serverIp!),
                          ),
                        ),
                      ],
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
                      icon: Icon(_isListening ? Icons.mic_off : Icons.mic, color: darkGrayText),
                      onPressed: _toggleListening,
                      tooltip: '궁금한 점을 질문하세요',
                    ),
                    const SizedBox(width: 8),
                    Expanded(
                      child: Text(
                        _voiceCommandStatusText,
                        style: const TextStyle(fontSize: 16, color: lightGrayText),
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
