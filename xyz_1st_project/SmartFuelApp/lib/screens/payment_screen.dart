import 'package:flutter/material.dart';
import 'dart:convert';
import 'package:http/http.dart' as http;
import 'package:provider/provider.dart';
import 'fuel_progress_screen.dart';
import 'dart:math';
import '../widgets/voice_command_bar.dart';
import '../services/voice_interaction_service.dart';
import '../config/app_config.dart';
import '../services/llm_service.dart';
import '../theme/app_theme.dart';

/// 결제 수단을 선택하고 결제를 진행하는 화면
class PaymentScreen extends StatefulWidget {
  final String fuelType;
  final int amount;

  const PaymentScreen({Key? key, required this.fuelType, required this.amount})
      : super(key: key);

  @override
  State<PaymentScreen> createState() => _PaymentScreenStateWrapper();
}

/// VoiceInteractionService를 하위 위젯에 제공하기 위한 래퍼
class _PaymentScreenStateWrapper extends State<PaymentScreen> {
  @override
  Widget build(BuildContext context) {
    return ChangeNotifierProvider(
      create: (_) => VoiceInteractionService(),
      child: _PaymentScreenContent(
        fuelType: widget.fuelType,
        amount: widget.amount,
      ),
    );
  }
}

/// 결제 화면의 실제 UI와 상태를 포함하는 위젯
class _PaymentScreenContent extends StatefulWidget {
  final String fuelType;
  final int amount;

  const _PaymentScreenContent({required this.fuelType, required this.amount});

  @override
  State<_PaymentScreenContent> createState() => _PaymentScreenState();
}

/// 결제 화면의 상태 관리 로직 (결제 수단 선택, 음성 명령 처리, 결제 시뮬레이션)
class _PaymentScreenState extends State<_PaymentScreenContent> {
  bool isProcessing = false;
  String _selectedPaymentMethod = '신용카드';
  int _selectedCardIndex = 0;
  final List<String> _cardNumbers = [];
  bool _isAwaitingConfirmation = false; 
  late VoiceInteractionService _voiceService;

  /// 위젯 초기화
  @override
  void initState() {
    super.initState();
    _voiceService = Provider.of<VoiceInteractionService>(context, listen: false);
    _voiceService.onResult = _processVoiceCommand;

    for (int i = 0; i < 5; i++) {
      _cardNumbers.add((Random().nextInt(9000) + 1000).toString());
    }

    WidgetsBinding.instance.addPostFrameCallback((_) {
      _voiceService.speakAndListen('결제 수단을 말씀해주세요.');
    });
  }

  /// 사용자 음성 명령 처리
  Future<void> _processVoiceCommand(String command) async {
    if (command.isEmpty) return;

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

      if (intent == 'no_help_needed') {
        _voiceService.deactivateFeature("네, 직접 선택해주세요.");
        return;
      }
      
      if (_isAwaitingConfirmation && confirmation == 'positive') {
        setState(() => _isAwaitingConfirmation = false);
        await simulatePayment();
        return;
      }

      if (method == null && confirmation == null) {
        throw Exception("결제 수단을 인식하지 못했습니다.");
      }

      if (method == null) {
        return;
      }

      String displayText = method;

      if (method == '신용카드') {
        if (index != null && index >= 0 && index < _cardNumbers.length) {
          _selectedCardIndex = index;
          displayText = '신용카드 ${index + 1}';
        } else {
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
        _voiceService.completeProcessing(displayText);
      });

      if (shouldPay) {
        await simulatePayment();
      } else if (method != null) {
        _isAwaitingConfirmation = true;
        final confirmationQuestion =
            '${_selectedPaymentMethod == '신용카드' ? '신용카드 ${_selectedCardIndex + 1}' : _selectedPaymentMethod}(으)로 결제할까요?';
        _voiceService.speakAndListen(confirmationQuestion);
      }
    } on ApiException catch (e) {
      debugPrint('LLM API 오류: $e');
      const errorMessage = '서버에 일시적인 문제가 발생했어요. 잠시 후 다시 시도해주세요.';
      if (mounted) _voiceService.speakAndListen(errorMessage);
    } catch (e) {
      debugPrint('LLM 결제수단 분석 오류: $e');
      const errorMessage = '죄송해요, 잘 이해하지 못했어요. 다시 말씀해주시겠어요?';
      if (mounted) _voiceService.speakAndListen(errorMessage);
    }
  }

  /// 결제 시뮬레이션 및 서버에 주유 시작 요청
  Future<void> simulatePayment() async {
    await _voiceService.speak("결제를 시작합니다.");
    setState(() => isProcessing = true);
    await Future.delayed(const Duration(seconds: 2)); 

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
        await _voiceService.speak("결제를 완료했습니다.");

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

  /// 카드 목록의 개별 카드 아이템 UI 생성
  Widget _buildCardItem(int index) {
    final isSelected = _selectedCardIndex == index;
    final cardGradients = [
      AppColors.cardGradientGrey,
      AppColors.cardGradientBlue,
      AppColors.cardGradientOrange,
      AppColors.cardGradientGreen,
      AppColors.cardGradientPurple,
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
          border: isSelected ? Border.all(color: AppColors.primary, width: 3) : null,
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

  /// 스크롤 가능한 카드 목록 UI 생성
  Widget _buildCardList() {
    return SizedBox(
      height: 150,
      child: ListView.builder(
        scrollDirection: Axis.horizontal,
        itemCount: 5,
        itemBuilder: (context, index) {
          final voiceService = Provider.of<VoiceInteractionService>(context);
          return GestureDetector(
            onTap: voiceService.isProcessing ? null : () {
              setState(() {
                _selectedCardIndex = index;
                _selectedPaymentMethod = '신용카드';
                voiceService.deactivateOnManualSelection();
              });
            },
            child: _buildCardItem(index),
          );
        },
      ),
    );
  }

  /// 간편 결제 버튼 그룹 UI 생성
  Widget _buildPayButtons() {
    final payMethods = {
      '네이버페이': {'bg': AppColors.naverPayBg, 'text': AppColors.naverPayText},
      '카카오페이': {'bg': AppColors.kakaoPayBg, 'text': AppColors.kakaoPayText},
      '토스페이': {'bg': AppColors.tossPayBg, 'text': AppColors.tossPayText},
    };

    return Row(
      children: payMethods.entries.map((entry) {
        final methodName = entry.key;
        final bgColor = entry.value['bg']!;
        final textColor = entry.value['text']!;
        final isSelected = _selectedPaymentMethod == methodName;
        final voiceService = Provider.of<VoiceInteractionService>(context);

        return Expanded(
          child: GestureDetector(
            onTap: voiceService.isProcessing ? null : () {
              setState(() {
                _selectedPaymentMethod = methodName;
                _selectedCardIndex = -1; 
                voiceService.deactivateOnManualSelection();
              });
            },
            child: AnimatedContainer(
              duration: const Duration(milliseconds: 200),
              height: 60,
              margin: const EdgeInsets.symmetric(horizontal: 4),
              decoration: BoxDecoration(
                color: bgColor,
                borderRadius: BorderRadius.circular(12),
                border: isSelected ? Border.all(color: AppColors.primary, width: 3) : null,
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

  /// 화면 UI 구성
  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);

    return Scaffold(
      appBar: AppBar(
        title: const Text('결제'),
      ),
      body: isProcessing
          ? Center(
              child: CircularProgressIndicator(
                color: theme.primaryColor,
              ),
            )
          : SingleChildScrollView(
              padding: const EdgeInsets.all(24.0),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    '${widget.amount}원',
                    style: theme.textTheme.displaySmall,
                  ),
                  const SizedBox(height: 8),
                  Text(
                    '${widget.fuelType} 주유를 진행합니다.', 
                    style: theme.textTheme.bodyLarge?.copyWith(color: AppColors.textSecondary),
                  ),
                  const SizedBox(height: 40),
                  VoiceCommandBar(
                    initialText: '결제 수단을 말씀해주세요.',
                    backgroundColor: AppColors.surface,
                    textColor: AppColors.textPrimary,
                  ),
                  const SizedBox(height: 40),
                  Text( 
                    '카드 선택',
                    style: theme.textTheme.titleLarge?.copyWith(fontSize: 20),
                  ),
                  const SizedBox(height: 16),
                  _buildCardList(), 
                  const SizedBox(height: 40),
                  Text( 
                    '간편 결제',
                    style: theme.textTheme.titleLarge?.copyWith(fontSize: 20),
                  ),
                  const SizedBox(height: 16),
                  _buildPayButtons(), 
                ],
              ),
            ),
      bottomNavigationBar: Padding(
        padding: const EdgeInsets.all(20.0),
        child: Consumer<VoiceInteractionService>(
          builder: (context, voiceService, child) => ElevatedButton(
          onPressed: isProcessing || voiceService.isProcessing ? null : simulatePayment,
          style: theme.elevatedButtonTheme.style,
          child: const Text('결제하기'),
        ),
        ),
      ),
    );
  }
}