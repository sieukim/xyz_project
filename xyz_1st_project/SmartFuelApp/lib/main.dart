import 'package:flutter/material.dart';
import 'package:kakao_flutter_sdk_user/kakao_flutter_sdk_user.dart';
import 'package:smart_fuel/config/app_config.dart';
import 'package:smart_fuel/theme/app_theme.dart';
import 'screens/splash_screen.dart';

Future<void> main() async {
  // Flutter 엔진과 위젯 바인딩을 보장합니다.
  WidgetsFlutterBinding.ensureInitialized();

  // .env 파일을 로드하고 설정값을 메모리에 올립니다.
  await AppConfig.load();

  // 카카오 SDK 초기화
  KakaoSdk.init(
    nativeAppKey: 'e3cdde95ba02edf430f38f688e02e0ed', // 실제 네이티브 앱 키
  );

  // 모든 초기화가 끝난 후 앱을 실행합니다.
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Smart Refueler',
      theme: AppTheme.getTheme(), // 중앙 관리되는 테마 적용
      home: const SplashScreen(),
      debugShowCheckedModeBanner: false,
    );
  }
}
