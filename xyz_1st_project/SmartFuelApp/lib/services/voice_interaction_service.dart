import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter_tts/flutter_tts.dart';
import 'package:speech_to_text/speech_to_text.dart';

/// 음성 인식(STT)과 음성 합성(TTS)의 상태를 정의
enum VoiceState { idle, listening, processing, speaking }

/// STT, TTS 기능을 통합 관리하고 UI에 상태 변화를 알리는 서비스
class VoiceInteractionService with ChangeNotifier {
  final FlutterTts _flutterTts = FlutterTts();
  final SpeechToText _speechToText = SpeechToText();

  VoiceState _state = VoiceState.idle;
  String _displayText = '';
  bool _isFeatureActive = true;

  Function(String)? onResult;

  VoiceState get state => _state;
  String get displayText => _displayText;
  bool get isFeatureActive => _isFeatureActive;
  bool get isProcessing => _state == VoiceState.listening || _state == VoiceState.processing || _state == VoiceState.speaking;

  /// 서비스 생성자, 초기화 로직 호출
  VoiceInteractionService() {
    _initialize();
  }

  /// TTS와 STT 엔진 초기화
  Future<void> _initialize() async {
    await _flutterTts.setLanguage('ko-KR');
    await _flutterTts.setSpeechRate(1.0);
    await _flutterTts.awaitSpeakCompletion(true);
    _setTtsHandlers();

    await _speechToText.initialize();
    notifyListeners();
  }

  /// TTS 이벤트 핸들러(시작, 종료) 설정
  void _setTtsHandlers() {
    _flutterTts.setStartHandler(() => _updateState(VoiceState.speaking));
    _flutterTts.setCompletionHandler(() {
      if (_state == VoiceState.speaking) {
        _updateState(VoiceState.idle);
      }
    });
  }

  /// 음성 서비스의 상태와 표시 텍스트를 업데이트하고 UI에 알림
  void _updateState(VoiceState newState, {String? text}) {
    _state = newState;
    if (text != null) {
      _displayText = text;
    }
    notifyListeners();
  }

  /// 텍스트를 읽어준 후 바로 음성 인식 시작
  Future<void> speakAndListen(String text) async {
    if (!_isFeatureActive || !_speechToText.isAvailable) return;
    _updateState(VoiceState.speaking, text: text);
    await _flutterTts.speak(text);
    startListening();
  }

  /// 텍스트를 음성으로 읽어줌
  Future<void> speak(String text) async {
    if (!_isFeatureActive) return;
    _updateState(VoiceState.speaking, text: text);
    await _flutterTts.speak(text);
  }

  /// 문자열을 한 글자씩 읽어준 후, 후속 텍스트를 읽고 음성 인식 시작
  Future<void> speakCharacterByCharacter(
      String characters, String followUpText) async {
    if (!_isFeatureActive || !_speechToText.isAvailable) return;
    
    _updateState(VoiceState.speaking, text: '$characters $followUpText');

    _flutterTts.setStartHandler(() {});
    _flutterTts.setCompletionHandler(() {});

    await _flutterTts.setSpeechRate(1.5); 
    for (final char in characters.split('')) {
      await _flutterTts.speak(char);
    }
    await _flutterTts.setSpeechRate(1.0); 
    await _flutterTts.speak(followUpText);
    _setTtsHandlers(); 

    startListening();
  }

  /// 음성 인식(STT) 시작
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

  /// 음성 인식 중지
  void stopListening() {
    _speechToText.stop();
    _updateState(VoiceState.idle, text: '');
  }

  /// 음성 인식 시작/중지 토글
  void toggleListening() {
    if (_state == VoiceState.listening) {
      stopListening();
    } else {
      startListening();
    }
  }

  /// 음성 기능을 비활성화하고 작별 인사
  void deactivateFeature(String farewellMessage) {
    _isFeatureActive = false;
    speak(farewellMessage);
    _updateState(VoiceState.idle, text: '음성 기능이 비활성화되었습니다.');
  }

  /// 사용자가 수동으로 UI를 조작했을 때 음성 기능 비활성화
  void deactivateOnManualSelection() {
    // 이미 비활성화 상태이면 아무것도 하지 않음
    if (!_isFeatureActive) return;

    _isFeatureActive = false;
    // 진행 중인 음성을 중지하고 상태를 업데이트합니다.
    _flutterTts.stop();
    _updateState(VoiceState.idle, text: '음성 기능이 비활성화되었습니다.');
  }

  /// 음성을 즉시 중지하고 음성 기능을 비활성화
  void stopAndDeactivate() {
    if (!_isFeatureActive) return;
    deactivateOnManualSelection();
  }

  /// 음성 명령 처리가 완료되었음을 알리고 결과 텍스트 표시
  void completeProcessing(String resultText) {
    _updateState(VoiceState.idle, text: resultText);
  }
}