import 'package:flutter/foundation.dart';
import 'package:smart_fuel/services/llm_service.dart';
import 'package:smart_fuel/services/voice_interaction_service.dart';

/// 대화가 이루어지는 화면의 종류 정의
enum ConversationScreen {
  fuelSelection,
  payment,
  fuelProgress,
}

/// LLM의 분석 결과에 따라 수행할 동작(음성 답변, 화면 전환 등)을 정의
class ConversationAction {
  final String? speakText; 
  final bool shouldListenNext; 
  final Map<String, dynamic> stateUpdate; 
  ConversationAction({
    this.speakText,
    this.shouldListenNext = false,
    this.stateUpdate = const {},
  });
}

/// 사용자 발화를 분석하고, 현재 화면과 맥락에 맞는 대화 흐름을 관리하는 서비스
class ConversationManagerService {
  /// 음성 명령을 받아 화면과 맥락에 맞게 처리하고 액션을 반환
  Future<ConversationAction> processVoiceCommand({
    required String command,
    required ConversationScreen screen,
    Map<String, dynamic> context = const {},
  }) async {
    if (command.isEmpty) {
      return ConversationAction(); 
    }

    try {
      final prompt = _buildPrompt(screen, command, context);
      final result = await LlmService.generateContent(prompt);

      if (result['intent'] == 'no_help_needed') {
        return ConversationAction(
          speakText: result['response'] as String? ?? "네, 직접 조작해주세요.",
          stateUpdate: {'deactivate_voice': true},
        );
      }

      switch (screen) {
        case ConversationScreen.fuelSelection:
          return _handleFuelSelection(result, context);
        case ConversationScreen.payment:
          return _handlePayment(result, context);
        case ConversationScreen.fuelProgress:
          return _handleFuelProgress(result);
      }
    } on ApiException catch (e) {
      debugPrint('LLM API 오류: $e');
      return ConversationAction(
        speakText: '서버에 일시적인 문제가 발생했어요. 잠시 후 다시 시도해주세요.',
        shouldListenNext: true,
      );
    } catch (e) {
      debugPrint('LLM 처리 오류: $e');
      return ConversationAction(
        speakText: '죄송해요, 잘 이해하지 못했어요. 다시 말씀해주시겠어요?',
        shouldListenNext: true,
      );
    }
  }

  /// 화면과 맥락에 맞는 LLM 프롬프트를 생성
  String _buildPrompt(ConversationScreen screen, String command, Map<String, dynamic> context) {
    switch (screen) {
      case ConversationScreen.fuelSelection:
        return """
        사용자의 발화를 분석해서 JSON으로 반환해줘.
        1. **정보 추출**: 'fuelType': '휘발유', '경유', '전기' 중 하나. 'amount': 만원 단위 숫자(예: 50000). '가득'은 -1로 설정.
        2. **의도 파악**: 'intent': 'order' (주문), 'payment' (결제), 'confirmation_positive' (긍정), 'confirmation_negative' (부정), 'no_help_needed' (도움 거절) 중 하나.
        **규칙**: 요청에 없는 정보는 null로 설정. 결과는 항상 JSON 형식.
        **입력**: "$command"
        """;
      case ConversationScreen.payment:
        return """
        사용자의 결제 요청에서 결제 수단과 결제 실행 여부를 추출해줘.
        - 사용 가능한 결제 수단: '신용카드', '네이버페이', '카카오페이', '토스페이'.
        - '카드'는 '신용카드'로 인식. "N번째 카드"는 0부터 시작하는 인덱스로 알려줘. (예: "3번 카드" -> 2)
        - 카드 색상과 인덱스 매칭: '회색'/'검정색' -> 0, '파란색'/'하늘색'-> 1, '주황색'/'분홍색' -> 2, '초록색'/'노란색' -> 3, '보라색' -> 4.
        - 결제 실행 의도(예: '결제해줘')가 보이면 "performPayment": true 를 포함.
        - 긍정 답변(예: '응', '네')은 "confirmation": "positive" 를 포함.
        - 직접 조작 의사(예: '괜찮아')는 "intent": "no_help_needed" 를 포함.
        - 결과는 반드시 JSON 형식으로 반환.
        사용자 요청: "$command"
        """;
      case ConversationScreen.fuelProgress:
        final progress = context['progress'] ?? 0;
        final remainingSeconds = context['remainingSeconds'] ?? 0;
        return """
        당신은 주유 중인 운전자를 돕는 친절한 AI 비서입니다. 사용자의 질문에 대해 JSON 형식으로 답변을 생성해주세요.
        ### 현재 주유 상태: 진행률: $progress%, 남은 시간: $remainingSeconds초
        ### 지침:
        1. 사용자의 질문 의도를 파악하세요. '진행률'이나 '남은 시간' 질문이면 위 정보를 활용해 답변하세요.
        2. 도움이 필요 없다고 말하면(예: "아니", "없어"), 'intent' 필드에 'no_help_needed'를 포함시키세요.
        3. 그 외 모든 질문(날씨, 농담 등)에도 친절한 답변을 생성하세요.
        4. 생성된 답변은 'response' 필드에 담아 JSON으로 반환하세요.
        ### 사용자 질문: "$command"
        """;
    }
  }

  /// 주유 설정 화면의 대화 로직 처리
  ConversationAction _handleFuelSelection(Map<String, dynamic> result, Map<String, dynamic> context) {
    final intent = result['intent'] as String?;
    final currentContext = context['conversationContext'];

    if (currentContext == 'awaitingFinalConfirmation') {
      if (intent == 'confirmation_positive') {
        return ConversationAction(stateUpdate: {'navigate_to_payment': true});
      } else {
        return ConversationAction(
          speakText: '다시 말씀해주세요.',
          shouldListenNext: true,
          stateUpdate: {'reset_context': true},
        );
      }
    }

    final fuelType = result['fuelType'] as String? ?? context['fuelType'];
    final amount = result['amount'] as int? ?? context['amount'];
    final bool isInfoComplete = (fuelType != null) && (amount != null);

    if (isInfoComplete) {
      final speakAmount = (amount == -1) ? '가득' : '${amount ~/ 10000}만원';
      return ConversationAction(
        speakText: '$fuelType, $speakAmount 으로 결제할까요?',
        shouldListenNext: true,
        stateUpdate: {
          'fuelType': fuelType,
          'amount': amount,
          'set_context': 'awaitingFinalConfirmation'
        },
      );
    } else {
      String question;
      if (fuelType == null) {
        question = '유종을 말씀해주세요.';
      } else {
        question = '금액을 말씀해주세요.';
      }
      return ConversationAction(
        speakText: question,
        shouldListenNext: true,
        stateUpdate: {'fuelType': fuelType, 'amount': amount},
      );
    }
  }

  /// 결제 화면의 대화 로직 처리
  ConversationAction _handlePayment(Map<String, dynamic> result, Map<String, dynamic> context) {
    final confirmation = result['confirmation'] as String?;
    final isAwaiting = context['isAwaitingConfirmation'] == true;
    if (isAwaiting && confirmation == 'positive') {
      return ConversationAction(stateUpdate: {'perform_payment': true});
    }

    final method = result['paymentMethod'] as String?;
    if (method == null) {
      return ConversationAction(speakText: '결제 수단을 인식하지 못했어요. 다시 말씀해주시겠어요?', shouldListenNext: true);
    }

    final shouldPay = result['performPayment'] == true;
    if (shouldPay) {
      return ConversationAction(stateUpdate: {'paymentMethod': method, 'cardIndex': result['cardIndex'], 'perform_payment': true});
    } else {
      final cardIndex = result['cardIndex'] as int?;
      final speakText = (method == '신용카드' && cardIndex != null) ? '신용카드 ${cardIndex + 1}' : method;
      return ConversationAction(
        speakText: '$speakText(으)로 결제할까요?',
        shouldListenNext: true,
        stateUpdate: {'paymentMethod': method, 'cardIndex': cardIndex, 'await_confirmation': true},
      );
    }
  }

  /// 주유 진행 화면의 대화 로직 처리
  ConversationAction _handleFuelProgress(Map<String, dynamic> result) {
    final responseText = result['response'] as String?;
    if (responseText == null || responseText.isEmpty) {
      return ConversationAction(speakText: '죄송해요, 잘 이해하지 못했어요.', shouldListenNext: true);
    }
    return ConversationAction(speakText: responseText, shouldListenNext: true);
  }
}