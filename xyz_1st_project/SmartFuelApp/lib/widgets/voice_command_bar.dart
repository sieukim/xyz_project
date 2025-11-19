import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../services/voice_interaction_service.dart';

/// 음성 인식 상태를 표시하고 마이크 버튼을 제공하는 UI 컴포넌트
class VoiceCommandBar extends StatelessWidget {
  final String initialText;
  final Color backgroundColor;
  final Color textColor;

  const VoiceCommandBar({
    Key? key,
    required this.initialText,
    this.backgroundColor = const Color(0xFFF2F4F6),
    this.textColor = const Color(0xFF333D4B),
  }) : super(key: key);

  /// 화면 UI 구성
  @override
  Widget build(BuildContext context) {
    return Consumer<VoiceInteractionService>(
      builder: (context, voiceService, child) {
        final bool isMicOff = !voiceService.isFeatureActive || voiceService.isProcessing;
        final String displayText = voiceService.displayText.isEmpty
            ? initialText
            : voiceService.displayText;

        return Container(
          padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
          decoration: BoxDecoration(
            color: backgroundColor,
            borderRadius: BorderRadius.circular(12),
          ),
          child: Row(
            children: [
              IconButton(
                icon: Icon(
                  isMicOff ? Icons.mic_off : Icons.mic,
                  color: isMicOff ? Colors.grey : textColor,
                ),
                onPressed: isMicOff ? null : voiceService.toggleListening,
                tooltip: '음성으로 명령하기',
              ),
              const SizedBox(width: 8),
              Expanded(
                child: Text(
                  displayText,
                  style: TextStyle(fontSize: 16, color: textColor, fontWeight: FontWeight.w600),
                ),
              ),
            ],
          ),
        );
      },
    );
  }
}