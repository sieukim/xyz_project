import 'package:flutter/foundation.dart';
import 'package:smart_fuel/services/llm_service.dart';
import 'package:smart_fuel/services/voice_interaction_service.dart';

/// 대화가 이루어지는 화면의 종류를 정의합니다.
enum ConversationScreen {
  fuelSelection,
  payment,
  fuelProgress,
}

/// 대화의 결과로 UI가 수행해야 할 작업을 정의하는 클래스입니다.
class ConversationAction {
  final String? speakText; // AI가 사용자에게 말할 내용
  final bool shouldListenNext; // 말한 뒤에 다시 사용자의 입력을 기다릴지 여부
  final Map<String, dynamic> stateUpdate; // 화면의 상태를 변경하기 위한 데이터

  ConversationAction({
    this.speakText,
    this.shouldListenNext = false,
    this.stateUpdate = const {},
  });
}

/// 대화 시나리오를 중앙에서 관리하는 서비스 클래스입니다.
class ConversationManagerService {
  /// 음성 명령을 처리하고 그에 따른 UI 행동을 반환합니다.
  ///
  /// [command]는 사용자의 음성 인식 결과입니다.
  /// [screen]은 현재 대화가 이루어지는 화면의 종류입니다.
  /// [context]는 화면의 현재 상태 데이터입니다. (예: 주유 진행률, 선택된 카드 정보 등)
  Future<ConversationAction> processVoiceCommand({
    required String command,
    required ConversationScreen screen,
    Map<String, dynamic> context = const {},
  }) async {
    if (command.isEmpty) {
      return ConversationAction(); // 아무것도 하지 않음
    }

    try {
      final prompt = _buildPrompt(screen, command, context);
      final result = await LlmService.generateContent(prompt);

      // 공통 의도 처리 (예: 음성 도움 거절)
      if (result['intent'] == 'no_help_needed') {
        return ConversationAction(
          speakText: result['response'] as String? ?? "네, 직접 조작해주세요.",
          stateUpdate: {'deactivate_voice': true},
        );
      }

      // 각 화면별 특화 로직 처리
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

  /// 화면 종류에 맞는 LLM 프롬프트를 생성합니다.
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

  // --- 각 화면별 응답 처리 로직 ---

  ConversationAction _handleFuelSelection(Map<String, dynamic> result, Map<String, dynamic> context) {
    final intent = result['intent'] as String?;
    final currentContext = context['conversationContext'];

    // 최종 확인 단계에서의 응답 처리
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

    // 정보가 모두 채워졌는지 확인
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
      // 부족한 정보 요청
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

  ConversationAction _handleFuelProgress(Map<String, dynamic> result) {
    final responseText = result['response'] as String?;
    if (responseText == null || responseText.isEmpty) {
      return ConversationAction(speakText: '죄송해요, 잘 이해하지 못했어요.', shouldListenNext: true);
    }
    return ConversationAction(speakText: responseText, shouldListenNext: true);
  }
}