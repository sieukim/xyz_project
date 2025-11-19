import 'package:flutter/material.dart';
import 'dart:convert';
import 'package:http/http.dart' as http;
import 'package:flutter_tts/flutter_tts.dart';
import 'package:speech_to_text/speech_to_text.dart';
import 'fuel_progress_screen.dart';
import 'dart:math';
import '../config/app_config.dart';
import '../services/llm_service.dart';

class PaymentScreen extends StatefulWidget {
  final String fuelType;
  final int amount;

  const PaymentScreen({Key? key, required this.fuelType, required this.amount})
      : super(key: key);

  @override
  State<PaymentScreen> createState() => _PaymentScreenState();
}

class _PaymentScreenState extends State<PaymentScreen> {
  final FlutterTts _flutterTts = FlutterTts();
  final SpeechToText _speechToText = SpeechToText();
  bool _isListening = false;
  String _recognizedWords = '';
  bool isProcessing = false;
  String _selectedPaymentMethod = '신용카드';
  int _selectedCardIndex = 0;
  final List<String> _cardNumbers = [];
  bool _isAwaitingConfirmation = false; // 결제 확인 대기 상태

  bool _voiceFeatureActive = true; // 음성 기능 활성화 여부
  @override
  void initState() {
    super.initState();
    _initTts();
    _initStt().then((_) {
      // STT 초기화 후, 화면이 완전히 빌드된 다음 음성 안내 및 인식 시작
      WidgetsBinding.instance.addPostFrameCallback((_) => _speakAndListen('결제 수단을 말씀해주세요.'));
    });

    for (int i = 0; i < 5; i++) {
      _cardNumbers.add((Random().nextInt(9000) + 1000).toString());
    }
  }

  Future<void> _initTts() async {
    await _flutterTts.setLanguage('ko-KR');
    await _flutterTts.setSpeechRate(1.0);
  }

  Future<void> _initStt() async {
    await _speechToText.initialize(
      onError: (error) {
        debugPrint('STT Error: ${error.errorMsg}');
        if (mounted) {
          setState(() {
            _recognizedWords = '음성 인식 오류가 발생했습니다.';
            _isListening = false;
          });
        }
      },
      onStatus: (status) {}, // UI 깜빡임 방지를 위해 onStatus에서 상태 변경 안 함
    );
  }

  /// TTS로 문장을 말하고, 끝나면 바로 음성 인식을 시작하는 헬퍼 함수
  Future<void> _speakAndListen(String text) async {
    if (!mounted) return;
    setState(() => _recognizedWords = text); // 화면에 현재 상태 표시
    await _flutterTts.speak(text);
    if (mounted) _startListening();
  }

  void _toggleListening() {
    if (!_speechToText.isAvailable) return;
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
      _recognizedWords = '듣는 중...';
    });
      _speechToText.listen(
        onResult: (result) {
          if (result.finalResult) {
            _processVoiceCommand(result.recognizedWords);
          }
        },
        localeId: 'ko_KR',
        listenFor: const Duration(seconds: 3), // 3초로 설정
      );
    }

  void _stopListening() {
    _speechToText.stop();
    if (!mounted) return;
    setState(() {
      _isListening = false;
      _recognizedWords = '';
    });
  }

  Future<void> _processVoiceCommand(String command) async {
    if (command.isEmpty) return;
    setState(() {
      _isListening = false;
      _recognizedWords = '분석 중...';
    });

    final prompt = """
    사용자의 결제 요청에서 결제 수단과 결제 실행 여부를 추출해줘.
    - 사용 가능한 결제 수단: '신용카드', '네이버페이', '카카오페이', '토스페이'.
    - 사용자가 '카드'라고 말하면 '신용카드'로 인식해줘.
    - 사용자가 "N번째 카드" 또는 "N번 카드"라고 말하면, 카드 인덱스를 0부터 시작하는 숫자로 알려줘. (예: "3번 카드" -> 2)
    - "가운데 카드"는 인덱스 2로 처리해줘.
    - "마지막 카드"는 인덱스 4로 처리해줘.    
    - 카드 색상과 인덱스 매칭: '회색'/'검정색' -> 0, '파란색'/'하늘색'-> 1, '주황색'/'분홍색' -> 2, '초록색'/'노란색' -> 3, '보라색' -> 4.
    - 사용자가 '결제', '해줘', '할게' 등 결제를 실행하려는 의도를 보이면 "performPayment": true 를 포함해줘.
    - 사용자가 "응", "네", "맞아" 등 긍정적인 답변을 하면 "confirmation": "positive" 를 포함해줘.
    - 사용자가 "괜찮아", "내가 할게" 등 직접 조작 의사를 보이면 "intent": "no_help_needed" 를 포함해줘.
    - 결과는 반드시 JSON 형식으로 반환해줘.
    - 예시 1: "카카오페이" -> {"paymentMethod": "카카오페이"}
    - 예시 2: "세 번째 카드로 할게" -> {"paymentMethod": "신용카드", "cardIndex": 2, "performPayment": true}
    - 예시 3: "가운데 카드로 결제해줘" -> {"paymentMethod": "신용카드", "cardIndex": 2, "performPayment": true}
    - 예시 4: "파란색 카드로 해줘" -> {"paymentMethod": "신용카드", "cardIndex": 1, "performPayment": true}
    - 예시 5: "응" -> {"confirmation": "positive"}
    - 알 수 없다면 null 값을 사용해줘.

    사용자 요청: "$command"
    """;

    try {
      final result = await LlmService.generateContent(prompt);
      final method = result['paymentMethod'] as String?;
      final index = result['cardIndex'] as int?;
      final shouldPay = result['performPayment'] == true;
      final confirmation = result['confirmation'] as String?;
      final intent = result['intent'] as String?;

      // 0. 사용자가 음성 도움을 거절한 경우
      if (intent == 'no_help_needed') {
        setState(() {
          _voiceFeatureActive = false;
          // 상태 텍스트를 변경하여 '분석 중...'에서 벗어납니다.
          _recognizedWords = '음성 기능이 비활성화되었습니다.';
        });
        await _flutterTts.speak("네, 직접 선택해주세요.");
        return; // ★★★ 함수를 즉시 종료합니다.
      }
      // 1. 결제 확인 대기 상태에서 긍정 답변을 받았을 경우
      if (_isAwaitingConfirmation && confirmation == 'positive') {
        setState(() => _isAwaitingConfirmation = false);
        await simulatePayment();
        return;
      }

      if (method == null && confirmation == null) {
        throw Exception("결제 수단을 인식하지 못했습니다.");
      }

      // `method`가 null이 아닐 때만 로직을 진행합니다.
      if (method == null) {
        return; // 또는 다른 오류 처리
      }

      String displayText = method;

      if (method == '신용카드') {
        if (index != null && index >= 0 && index < _cardNumbers.length) {
          // LLM이 반환한 인덱스로 카드 선택
          _selectedCardIndex = index;
          displayText = '신용카드 ${index + 1}';
        } else {
          // '신용카드'라고만 말한 경우, 첫 번째 카드를 기본으로 선택
          _selectedCardIndex = 0;
          displayText = '신용카드 1';
        }
      }

      if (shouldPay) {
        displayText += '으로 결제합니다.';
      }

      setState(() {
        _selectedPaymentMethod = method;
        if (method == '신용카드') {
          _selectedCardIndex = index ?? _selectedCardIndex;
        } else {
          _selectedCardIndex = -1;
        }
        _recognizedWords = displayText;
      });

      if (shouldPay) {
        await simulatePayment();
      } else if (method != null) {
        // 결제 수단만 언급된 경우, 되물어보기
        _isAwaitingConfirmation = true;
        final confirmationQuestion =
            '${_selectedPaymentMethod == '신용카드' ? '신용카드 ${_selectedCardIndex + 1}' : _selectedPaymentMethod}(으)로 결제할까요?';
        setState(() => _recognizedWords = confirmationQuestion);
        await _flutterTts.speak(confirmationQuestion);
        if (mounted) _startListening();
      }
    } on ApiException catch (e) {
      debugPrint('LLM API 오류: $e');
      const errorMessage = '서버에 일시적인 문제가 발생했어요. 잠시 후 다시 시도해주세요.';
      if (mounted) await _speakAndListen(errorMessage);
    } catch (e) {
      debugPrint('LLM 결제수단 분석 오류: $e');
      const errorMessage = '죄송해요, 잘 이해하지 못했어요. 다시 말씀해주시겠어요?';
      if (mounted) await _speakAndListen(errorMessage);
    }
  }

  /// 음성 인식 및 분석이 진행 중인지 여부를 반환합니다.
  bool get _isVoiceProcessing => _isListening || _recognizedWords == '분석 중...';

  Future<void> simulatePayment() async {
    await _flutterTts.speak("결제를 시작합니다.");

    setState(() => isProcessing = true);

    await Future.delayed(const Duration(seconds: 2)); // 가상 결제 지연

    try {
      final orderId =
          '${DateTime.now().millisecondsSinceEpoch}-${Random().nextInt(900000)}';
      final payload = {
        'event': 'payment_complete',
        'orderId': orderId,
        'fuelType': widget.fuelType,
        'amount': widget.amount,
        'paymentMethod': _selectedPaymentMethod,
        'source': 'mobile_app',
      };

      final res = await http
          .post(
            Uri.parse('${AppConfig.rosBaseUrl}/start_fuel'),
            headers: {'Content-Type': 'application/json'},
            body: jsonEncode(payload),
          )
          .timeout(const Duration(seconds: 8));

      if (res.statusCode == 200) {
        await _flutterTts.speak("결제를 완료했습니다.");

        String returnedOrderId = orderId;
        try {
          final body = jsonDecode(res.body);
          if (body is Map && body['orderId'] != null) {
            returnedOrderId = body['orderId'].toString();
          }
        } catch (_) {}

        if (!mounted) return;
        Navigator.pushReplacement(
          context,
          MaterialPageRoute(
            builder: (_) => FuelProgressScreen(
              orderId: returnedOrderId,
              rosBaseUrl: AppConfig.rosBaseUrl,
            ),
          ),
        );
      } else {
        throw Exception('서버 오류 (${res.statusCode})');
      }
    } catch (e) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('결제 실패: $e')),
      );
    } finally {
      setState(() => isProcessing = false);
    }
  }

  Widget _buildCardItem(int index) {
    final isSelected = _selectedCardIndex == index;
    final cardGradients = [
      [const Color(0xFF6B7684), const Color(0xFF333D4B)], // Grey
      [const Color(0xFF0052D4), const Color(0xFF4364F7), const Color(0xFF6FB1FC)], // Blue
      [const Color(0xFFD4145A), const Color(0xFFFBB03B)], // Orange/Pink
      [const Color(0xFF009245), const Color(0xFFFCEE21)], // Green/Yellow
      [const Color(0xFF4776E6), const Color(0xFF8E54E9)], // Purple/Blue
    ];

    return Transform.scale(
      scale: isSelected ? 1.0 : 0.95,
      child: Container(
        width: 200,
        margin: const EdgeInsets.symmetric(horizontal: 8),
        decoration: BoxDecoration(
          gradient: LinearGradient(
            colors: cardGradients[index % cardGradients.length],
            begin: Alignment.topLeft,
            end: Alignment.bottomRight,
          ),
          borderRadius: BorderRadius.circular(12),
          border: isSelected ? Border.all(color: const Color(0xFF3182F7), width: 3) : null,
          boxShadow: [
            BoxShadow(
              color: Colors.black.withOpacity(0.2),
              blurRadius: 8,
              offset: const Offset(0, 4),
            ),
          ],
        ),
        child: Padding(
          padding: const EdgeInsets.all(16.0),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text(
                'My Card ${index + 1}',
                style: const TextStyle(color: Colors.white, fontSize: 16, fontWeight: FontWeight.bold),
              ),
              const Spacer(),
              Text(
                '**** **** **** ${_cardNumbers[index]}',
                style: const TextStyle(color: Colors.white70, fontSize: 14, letterSpacing: 1.5),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildCardList() {
    return SizedBox(
      height: 150,
      child: ListView.builder(
        scrollDirection: Axis.horizontal,
        itemCount: 5,
        itemBuilder: (context, index) {
          return GestureDetector(
            onTap: _isVoiceProcessing ? null : () {
              setState(() {
                _selectedCardIndex = index;
                _selectedPaymentMethod = '신용카드';
                if (_voiceFeatureActive) _voiceFeatureActive = false;
              });
            },
            child: _buildCardItem(index),
          );
        },
      ),
    );
  }

  Widget _buildPayButtons() {
    final payMethods = {
      '네이버페이': {'bg': const Color(0xFFE6F8F0), 'text': const Color(0xFF03C75A)},
      '카카오페이': {'bg': const Color(0xFFFFFBE6), 'text': const Color(0xFF3C1E1E)},
      '토스페이': {'bg': const Color(0xFFE6F0FF), 'text': const Color(0xFF0064FF)},
    };

    return Row(
      children: payMethods.entries.map((entry) {
        final methodName = entry.key;
        final bgColor = entry.value['bg']!;
        final textColor = entry.value['text']!;
        final isSelected = _selectedPaymentMethod == methodName;

        return Expanded(
          child: GestureDetector(
            onTap: _isVoiceProcessing ? null : () {
              setState(() {
                _selectedPaymentMethod = methodName;
                _selectedCardIndex = -1; // Deselect card
                if (_voiceFeatureActive) _voiceFeatureActive = false;
              });
            },
            child: AnimatedContainer(
              duration: const Duration(milliseconds: 200),
              height: 60,
              margin: const EdgeInsets.symmetric(horizontal: 4),
              decoration: BoxDecoration(
                color: bgColor,
                borderRadius: BorderRadius.circular(12),
                border: isSelected ? Border.all(color: const Color(0xFF3182F7), width: 3) : null,
              ),
              child: Center(
                child: Text(
                  methodName,
                  style: TextStyle(
                    fontSize: 15,
                    fontWeight: FontWeight.bold,
                    color: textColor,
                  ),
                ), 
              ),
            ),
          ),
        );
      }).toList(),
    );
  }

  @override
  Widget build(BuildContext context) {
    const tossBlue = Color(0xFF3182F7);
    const darkGrayText = Color(0xFF333D4B);
    const lightGrayText = Color(0xFF6B7684);
    const white = Colors.white;

    return Scaffold(
      backgroundColor: white, 
      appBar: AppBar(
        title: const Text('결제', style: TextStyle(color: darkGrayText)),
        backgroundColor: white,
        elevation: 0,
        iconTheme: const IconThemeData(color: darkGrayText),
      ),
      body: isProcessing
          ? Center(
              child: CircularProgressIndicator(
                color: tossBlue,
              ),
            )
          : SingleChildScrollView(
              padding: const EdgeInsets.all(24.0),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    '${widget.amount}원',
                    style: const TextStyle(
                      fontSize: 34,
                      fontWeight: FontWeight.bold,
                      color: darkGrayText,
                    ),
                  ),
                  const SizedBox(height: 8),
                  Text(
                    '${widget.fuelType} 주유를 진행합니다.', 
                    style: const TextStyle(
                      fontSize: 20,
                      color: lightGrayText,
                    ),
                  ),
                  const SizedBox(height: 40),
                  Container(
                    padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
                    decoration: BoxDecoration(
                      color: const Color(0xFFF2F4F6),
                      borderRadius: BorderRadius.circular(12),
                    ),
                    child: Row(
                      children: [
                        IconButton(
                          icon: Icon(
                          !_voiceFeatureActive || _isVoiceProcessing ? Icons.mic_off : Icons.mic,
                          color: !_voiceFeatureActive || _isVoiceProcessing ? Colors.grey : darkGrayText,
                          ),
                        onPressed:
                            !_voiceFeatureActive || isProcessing || _isVoiceProcessing ? null : _toggleListening,
                          tooltip: '음성으로 결제수단 선택',
                        ),
                        const SizedBox(width: 8),
                        Expanded(
                          child: Text(
                            _recognizedWords.isNotEmpty ? _recognizedWords : '결제 수단을 말씀해주세요.',
                            style: const TextStyle(fontSize: 16, color: darkGrayText, fontWeight: FontWeight.w600),
                          ),
                        ),
                      ],
                    ),
                  ),
                  const SizedBox(height: 40),
                  const Text( // '카드 선택' 제목
                    '카드 선택',
                    style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold, color: darkGrayText),
                  ),
                  const SizedBox(height: 16),
                  _buildCardList(), // 카드 목록 위젯
                  const SizedBox(height: 40),
                  const Text( // '간편 결제' 제목
                    '간편 결제',
                    style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold, color: darkGrayText),
                  ),
                  const SizedBox(height: 16),
                  _buildPayButtons(), // 간편 결제 버튼 위젯
                ],
              ),
            ),
      bottomNavigationBar: Padding(
        padding: const EdgeInsets.all(20.0),
        child: ElevatedButton(
          onPressed: isProcessing || _isVoiceProcessing ? null : simulatePayment,
          style: ElevatedButton.styleFrom(
            backgroundColor: tossBlue,
            padding: const EdgeInsets.symmetric(vertical: 16),
            shape: RoundedRectangleBorder(
              borderRadius: BorderRadius.circular(12),
            ),
            elevation: 0,
            disabledBackgroundColor: Colors.grey[300],
          ),
          child: const Text(
            '결제하기',
            style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold, color: Colors.white),
          ),
        ),
      ),
    );
  }
}