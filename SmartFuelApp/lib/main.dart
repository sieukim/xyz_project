import 'package:flutter/material.dart';
import 'package:kakao_flutter_sdk_user/kakao_flutter_sdk_user.dart';
import 'screens/splash_screen.dart';

void main() {
  // 카카오 SDK 초기화
  KakaoSdk.init(
    nativeAppKey: 'e3cdde95ba02edf430f38f688e02e0ed', // 실제 네이티브 앱 키
    javaScriptAppKey:
        'bc562e3c3555eef9ec1d6e51b0016b3c', // JavaScript 키가 다르다면 별도로 설정
  );

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
