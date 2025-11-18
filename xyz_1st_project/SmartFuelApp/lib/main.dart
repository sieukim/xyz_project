import 'package:flutter/material.dart';
import 'package:kakao_flutter_sdk_user/kakao_flutter_sdk_user.dart';
import 'package:smart_fuel/config/app_config.dart';
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
      title: 'SmartFuel',
      theme: ThemeData(
        useMaterial3: true,
        fontFamily: 'NotoSansKR', // 한글 폰트 (옵션)
        // 전체 배경을 연한 그레이 계열로 설정하고, 기본 색상은 차분한 톤으로 지정
        colorScheme: const ColorScheme.light(
          primary: Color(0xFF607D8B), // blueGrey tone for primary actions
          background: Color(0xFFF5F7FA),
          surface: Colors.white,
          onPrimary: Colors.white,
        ),
        scaffoldBackgroundColor: const Color(0xFFF5F7FA),
        cardColor: Colors.white, dialogTheme: DialogThemeData(backgroundColor: Colors.white),
      ),
      home: const SplashScreen(),
      debugShowCheckedModeBanner: false,
    );
  }
}
