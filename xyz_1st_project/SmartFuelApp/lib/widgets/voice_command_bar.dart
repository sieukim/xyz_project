import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../services/voice_interaction_service.dart';

/// 음성 인식 상태를 표시하고 제어하는 재사용 가능한 위젯입니다.
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

  @override
  Widget build(BuildContext context) {
    // Provider를 통해 VoiceInteractionService에 접근합니다.
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