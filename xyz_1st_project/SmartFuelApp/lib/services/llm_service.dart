import 'dart:convert';
import 'package:flutter/foundation.dart';
import 'package:http/http.dart' as http;
import '../config/app_config.dart';

class LlmService {
  static String get _apiKey => AppConfig.geminiApiKey;
  static String get _modelEndpoint =>
      'https://generativelanguage.googleapis.com/v1/models/gemini-2.5-flash:generateContent?key=$_apiKey';

  /// LLM API를 호출하여 주어진 프롬프트를 기반으로 콘텐츠를 생성합니다.
  ///
  /// [prompt]는 LLM에게 보낼 전체 지침입니다.
  /// 반환값은 LLM이 생성한 JSON 응답을 파싱한 Map<String, dynamic>입니다.
  static Future<Map<String, dynamic>> generateContent(String prompt) async {
    final url = Uri.parse('$_modelEndpoint&alt=json');

    final response = await http.post(
      url,
      headers: {'Content-Type': 'application/json'},
      body: jsonEncode({
        'contents': [
          {'parts': [{'text': prompt}]}
        ],
        'safetySettings': [
          // 모든 안전 설정을 가장 낮은 수준으로 설정하여 차단 가능성을 최소화합니다.
          // 실제 서비스에서는 적절한 수준으로 조절해야 합니다.
          {'category': 'HARM_CATEGORY_HARASSMENT', 'threshold': 'BLOCK_NONE'},
          {'category': 'HARM_CATEGORY_HATE_SPEECH', 'threshold': 'BLOCK_NONE'},
          {'category': 'HARM_CATEGORY_SEXUALLY_EXPLICIT', 'threshold': 'BLOCK_NONE'},
          {
            'category': 'HARM_CATEGORY_DANGEROUS_CONTENT',
            'threshold': 'BLOCK_NONE'
          },
        ]
      }),
    );

    if (response.statusCode != 200) {
      throw Exception('API 요청 실패 (상태 코드: ${response.statusCode}): ${response.body}');
    }

    final responseBody = jsonDecode(utf8.decode(response.bodyBytes));

    if (responseBody['candidates'] == null || (responseBody['candidates'] as List).isEmpty) {
      final error = responseBody['error'];
      if (error != null) throw Exception('API 오류: ${error['message']}');
      throw Exception('API로부터 유효한 응답을 받지 못했습니다.');
    }

    final content = responseBody['candidates'][0]['content']['parts'][0]['text'] as String;
    
    // LLM 응답에서 JSON 객체만 추출 (마크다운, 일반 텍스트 등 불필요한 부분 제거)
    final jsonRegex = RegExp(r'\{[\s\S]*\}');
    final match = jsonRegex.firstMatch(content);

    if (match == null) {
      debugPrint('LLM 응답에서 JSON을 찾을 수 없습니다. 응답: $content');
      throw const FormatException('LLM 응답에서 유효한 JSON 객체를 찾을 수 없습니다.');
    }

    return jsonDecode(match.group(0)!) as Map<String, dynamic>;
  }

  /// LLM API를 호출하여 주어진 프롬프트를 기반으로 텍스트 콘텐츠만 생성합니다.
  ///
  /// JSON 파싱을 시도하지 않고 순수 텍스트를 반환합니다.
  /// [prompt]는 LLM에게 보낼 전체 지침입니다.
  /// 반환값은 LLM이 생성한 String입니다.
  static Future<String> generateTextOnly(String prompt) async {
    final url = Uri.parse('$_modelEndpoint&alt=json');

    final response = await http.post(
      url,
      headers: {'Content-Type': 'application/json'},
      body: jsonEncode({
        'contents': [
          {'parts': [{'text': prompt}]}
        ]
      }),
    );

    if (response.statusCode != 200) {
      throw Exception('API 요청 실패 (상태 코드: ${response.statusCode}): ${response.body}');
    }

    final responseBody = jsonDecode(utf8.decode(response.bodyBytes));

    if (responseBody['candidates'] == null || (responseBody['candidates'] as List).isEmpty) {
      final finishReason = responseBody['promptFeedback']?['blockReason'];
      if (finishReason != null) throw Exception('API가 응답을 차단했습니다. 이유: $finishReason');
      throw Exception('API로부터 유효한 응답을 받지 못했습니다.');
    }

    return responseBody['candidates'][0]['content']['parts'][0]['text'] as String;
  }
}