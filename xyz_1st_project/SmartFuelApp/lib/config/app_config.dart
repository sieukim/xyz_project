import 'package:flutter_dotenv/flutter_dotenv.dart';

/// .env 파일의 환경 변수를 관리하는 클래스
class AppConfig {
  /// 앱 시작 시 .env 파일을 로드합니다.
  static Future<void> load() async {
    await dotenv.load(fileName: ".env");
  }

  static String get rosIp => dotenv.get('ROS_IP', fallback: '192.168.50.211');
  static int get rosApiPort => int.tryParse(dotenv.get('ROS_API_PORT', fallback: '8000')) ?? 8000;
  static int get webcamStreamerPort => int.tryParse(dotenv.get('WEBCAM_STREAMER_PORT', fallback: '8081')) ?? 8081;
  static int get realsenseStreamerPort => int.tryParse(dotenv.get('REALSENSE_STREAMER_PORT', fallback: '8082')) ?? 8082;
  static int get ocrStreamerPort => int.tryParse(dotenv.get('OCR_STREAMER_PORT', fallback: '8083')) ?? 8083;
  static int get totalFuelingSeconds => int.tryParse(dotenv.get('TOTAL_FUELING_SECONDS', fallback: '60')) ?? 60;
  static String get geminiApiKey => dotenv.get('GEMINI_API_KEY');
  static String get googleSignInClientId => dotenv.get('GOOGLE_SIGNIN_CLIENT_ID');

  static String get rosBaseUrl => 'http://$rosIp:$rosApiPort';
  static String get ocrStreamerUrl => 'http://$rosIp:$ocrStreamerPort';
  static String get webcamStreamerUrl => 'http://$rosIp:$webcamStreamerPort';
  static String get realsenseStreamerUrl => 'http://$rosIp:$realsenseStreamerPort';
}