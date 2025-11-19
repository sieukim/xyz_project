import 'dart:convert';
import 'package:flutter/foundation.dart';
import 'package:http/http.dart' as http;
import '../config/app_config.dart';

/// LLM API 통신 중 발생하는 예외
class ApiException implements Exception {}

/// Gemini LLM API와 통신하여 콘텐츠를 생성하고 결과를 파싱하는 서비스
class LlmService {
  static String get _apiKey => AppConfig.geminiApiKey;
  static String get _modelEndpoint =>
      'https://generativelanguage.googleapis.com/v1/models/gemini-2.5-flash:generateContent?key=$_apiKey';

  /// 프롬프트를 전송하여 LLM으로부터 JSON 형식의 응답을 생성
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
      throw ApiException(); 
    }

    final responseBody = jsonDecode(utf8.decode(response.bodyBytes));

    if (responseBody['candidates'] == null || (responseBody['candidates'] as List).isEmpty) {
      final error = responseBody['error'];
      if (error != null) throw ApiException(); 
      throw Exception('API로부터 유효한 응답을 받지 못했습니다.');
    }

    final content = responseBody['candidates'][0]['content']['parts'][0]['text'] as String;
    
    final jsonRegex = RegExp(r'\{[\s\S]*\}');
    final match = jsonRegex.firstMatch(content);

    if (match == null) {
      debugPrint('LLM 응답에서 JSON을 찾을 수 없습니다. 응답: $content');
      throw const FormatException('LLM 응답에서 유효한 JSON 객체를 찾을 수 없습니다.');
    }

    return jsonDecode(match.group(0)!) as Map<String, dynamic>;
  }

  /// 프롬프트를 전송하여 LLM으로부터 텍스트 형식의 응답을 생성
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