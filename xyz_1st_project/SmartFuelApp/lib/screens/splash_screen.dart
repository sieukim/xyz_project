import 'package:flutter/material.dart';
import 'dart:async';
import 'login_screen.dart';
import '../services/kakao_login_service.dart';
import '../services/google_login_service.dart';
import 'fuel_selection_screen.dart';

class SplashScreen extends StatefulWidget {
  const SplashScreen({Key? key}) : super(key: key);

  @override
  State<SplashScreen> createState() => _SplashScreenState();
}

class _SplashScreenState extends State<SplashScreen> {
  @override
  void initState() {
    super.initState();
    _checkAndNavigate();
  }

  /// Delay splash for 3 seconds, then check login services and navigate.
  Future<void> _checkAndNavigate() async {
    debugPrint("로그인 x");
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (!mounted) return; 
      Navigator.pushReplacement(
          context,
          MaterialPageRoute(builder: (_) => const FuelSelectionScreen()),
        );
    });
    return;

    try {
      // 3초 대기
      await Future.delayed(const Duration(seconds: 3));
      if (!mounted) return;

      // 먼저 카카오 로그인 여부 확인
      bool isKakaoLoggedIn = false;
      try {
        isKakaoLoggedIn = await KakaoLoginService.instance.isLoggedIn();
      } catch (e) {
        debugPrint('카카오 로그인 체크 실패: $e');
      }

      // 구글 로그인 여부 확인
      bool isGoogleLoggedIn = false;
      try {
        isGoogleLoggedIn = GoogleLoginService.instance.isSignedIn;
      } catch (e) {
        debugPrint('구글 로그인 체크 실패: $e');
      }

      // mounted 체크
      if (!mounted) return;

      // 로그인 상태에 따라 화면 전환
      if (isKakaoLoggedIn || isGoogleLoggedIn) {
        Navigator.pushReplacement(
          context,
          MaterialPageRoute(builder: (_) => const FuelSelectionScreen()),
        );
      } else {
        Navigator.pushReplacement(
          context,
          MaterialPageRoute(builder: (_) => const LoginScreen()),
        );
      }
    } catch (e) {
      debugPrint('스플래시 화면 오류: $e');
      if (mounted) {
        // 오류 발생 시 로그인 화면으로 이동
        Navigator.pushReplacement(
          context,
          MaterialPageRoute(builder: (_) => const LoginScreen()),
        );
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    final media = MediaQuery.of(context);
    final double width = media.size.width;
    final double height = media.size.height;
    // 이미지와 텍스트 크기를 화면 크기에 비례하도록 설정
    final double imageWidth = (width * 0.45).clamp(120.0, 340.0);
    final double titleSize = (width * 0.07).clamp(18.0, 36.0);
    final double subtitleSize = (width * 0.04).clamp(12.0, 20.0);

    return Scaffold(
      backgroundColor: Colors.white,
      body: SafeArea(
        child: Center(
          child: Padding(
            padding: EdgeInsets.symmetric(horizontal: width * 0.06),
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              mainAxisSize: MainAxisSize.min,
              children: [
                // ✅ 주유 로봇 일러스트 이미지 (반응형)
                Image.asset(
                  'assets/images/refuel_robot.png', // 이미지 파일 경로
                  width: imageWidth,
                  fit: BoxFit.contain,
                ),
                SizedBox(height: height * 0.03),
                Text(
                  'Smart Refueler',
                  style: TextStyle(
                    fontSize: titleSize,
                    fontWeight: FontWeight.bold,
                    color: Colors.orangeAccent,
                  ),
                ),
                SizedBox(height: height * 0.01),
                Text(
                  '로봇이 주유 준비 중입니다...',
                  style: TextStyle(fontSize: subtitleSize, color: Colors.grey),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}
