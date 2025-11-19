import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter_tts/flutter_tts.dart';
import 'package:speech_to_text/speech_to_text.dart';

/// 음성 상호작용의 상태를 정의합니다.
enum VoiceState { idle, listening, processing, speaking }

/// STT, TTS 관련 로직을 캡슐화하고 UI에 상태 변경을 알리는 서비스 클래스입니다.
class VoiceInteractionService with ChangeNotifier {
  final FlutterTts _flutterTts = FlutterTts();
  final SpeechToText _speechToText = SpeechToText();

  VoiceState _state = VoiceState.idle;
  String _displayText = '';
  bool _isFeatureActive = true;

  // 외부에서 음성 인식 결과를 받아 처리할 콜백 함수
  Function(String)? onResult;

  // Getters
  VoiceState get state => _state;
  String get displayText => _displayText;
  bool get isFeatureActive => _isFeatureActive;
  bool get isProcessing => _state == VoiceState.listening || _state == VoiceState.processing || _state == VoiceState.speaking;

  VoiceInteractionService() {
    _initialize();
  }

  Future<void> _initialize() async {
    // TTS 초기화
    await _flutterTts.setLanguage('ko-KR');
    await _flutterTts.setSpeechRate(1.0);
    await _flutterTts.awaitSpeakCompletion(true);

    _flutterTts.setStartHandler(() => _updateState(VoiceState.speaking));
    _flutterTts.setCompletionHandler(() {
      if (_state == VoiceState.speaking) {
        _updateState(VoiceState.idle);
      }
    });

    // STT 초기화
    await _speechToText.initialize();
    notifyListeners();
  }

  void _updateState(VoiceState newState, {String? text}) {
    _state = newState;
    if (text != null) {
      _displayText = text;
    }
    notifyListeners();
  }

  /// 음성 안내 후 바로 듣기를 시작합니다.
  Future<void> speakAndListen(String text) async {
    if (!_isFeatureActive || !_speechToText.isAvailable) return;
    _updateState(VoiceState.speaking, text: text);
    await _flutterTts.speak(text);
    startListening();
  }

  /// 음성 안내만 합니다.
  Future<void> speak(String text) async {
    if (!_isFeatureActive) return;
    _updateState(VoiceState.speaking, text: text);
    await _flutterTts.speak(text);
  }

  /// 음성 인식을 시작합니다.
  void startListening() {
    if (!_isFeatureActive || !_speechToText.isAvailable || _state == VoiceState.listening) return;

    _updateState(VoiceState.listening, text: '듣는 중...');
    _speechToText.listen(
      onResult: (result) {
        if (result.finalResult) {
          _updateState(VoiceState.processing, text: '분석 중...');
          onResult?.call(result.recognizedWords);
        }
      },
      localeId: 'ko_KR',
      listenFor: const Duration(seconds: 3),
    );
  }

  /// 음성 인식을 중지합니다.
  void stopListening() {
    _speechToText.stop();
    _updateState(VoiceState.idle, text: '');
  }

  /// 마이크 버튼 토글 로직
  void toggleListening() {
    if (_state == VoiceState.listening) {
      stopListening();
    } else {
      startListening();
    }
  }

  /// 음성 기능을 비활성화합니다.
  void deactivateFeature(String farewellMessage) {
    _isFeatureActive = false;
    speak(farewellMessage);
    _updateState(VoiceState.idle, text: '음성 기능이 비활성화되었습니다.');
  }

  /// 사용자가 수동으로 UI를 조작했을 때 음성 기능을 비활성화하는 공통 메서드입니다.
  void deactivateOnManualSelection() {
    if (_isFeatureActive) {
      _isFeatureActive = false;
      speak("수동으로 선택하셨네요.");
      _updateState(VoiceState.idle, text: '음성 기능이 비활성화되었습니다.');
    }
  }
  /// LLM 처리 완료 후 서비스 상태를 업데이트합니다.
  void completeProcessing(String resultText) {
    _updateState(VoiceState.idle, text: resultText);
  }
}