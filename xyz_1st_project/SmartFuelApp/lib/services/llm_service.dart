import 'dart:convert';
import 'package:flutter/foundation.dart';
import 'package:http/http.dart' as http;

class LlmService {
  // ⚠️ 실제 앱에서는 API 키를 .env 파일 등으로 안전하게 관리해야 합니다.
  static const String _apiKey = 'AIzaSyBbqc2ZFo_1n8Un8vX7509u0au44SanMC8';
  static const String _modelEndpoint =
      'https://generativelanguage.googleapis.com/v1/models/gemini-2.5-flash:generateContent?key=$_apiKey';

  /// LLM API를 호출하여 주어진 프롬프트를 기반으로 콘텐츠를 생성합니다.
  ///
  /// [prompt]는 LLM에게 보낼 전체 지침입니다.
  /// 반환값은 LLM이 생성한 JSON 응답을 파싱한 Map<String, dynamic>입니다.
  static Future<Map<String, dynamic>> generateContent(String prompt) async {
    final url = Uri.parse(_modelEndpoint);

    final response = await http.post(
      url,
      headers: {'Content-Type': 'application/json'},
      body: jsonEncode({
        'contents': [{'parts': [{'text': prompt}]}]
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
    final jsonResult = jsonDecode(content.replaceAll('```json', '').replaceAll('```', '').trim());
    return jsonResult as Map<String, dynamic>;
  }
}