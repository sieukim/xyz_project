import 'package:flutter/material.dart';

/// 앱 전체에서 사용할 색상 팔레트를 정의
class AppColors { 
  // Core Colors (Toss Concept)
  static const primary = Color(0xFF3182F7); // 주요 색상 (Toss Blue)
  static const background = Color(0xFFFFFFFF); // 배경 색상 (White)
  static const surface = Color(0xFFF2F4F6); // Light Gray for cards, backgrounds
  static const textPrimary = Color(0xFF333D4B); // Main text color
  static const textSecondary = Color(0xFF6B7684); // Subdued text color
  
  // 결제 화면 카드 그라데이션
  static const cardGradientGrey = [Color(0xFF6B7684), Color(0xFF333D4B)];
  static const cardGradientBlue = [Color(0xFF0052D4), Color(0xFF4364F7), Color(0xFF6FB1FC)];
  static const cardGradientOrange = [Color(0xFFD4145A), Color(0xFFFBB03B)];
  static const cardGradientGreen = [Color(0xFF009245), Color(0xFFFCEE21)];
  static const cardGradientPurple = [Color(0xFF4776E6), Color(0xFF8E54E9)];

  // 결제 화면 간편결제 버튼 색상
  static const naverPayBg = Color(0xFFE6F8F0);
  static const naverPayText = Color(0xFF03C75A);
  static const kakaoPayBg = Color(0xFFFFFBE6);
  static const kakaoPayText = Color(0xFF3C1E1E);
  static const tossPayBg = Color(0xFFE6F0FF);
  static const tossPayText = Color(0xFF0064FF);
}

/// 앱의 전역 테마(색상, 폰트, 버튼 스타일 등)를 설정
class AppTheme {
  /// 앱의 ThemeData를 생성하여 반환
  static ThemeData getTheme() { 
    return ThemeData(
      useMaterial3: true, // Material 3 사용을 명시적으로 활성화합니다.
      primaryColor: AppColors.primary,
      scaffoldBackgroundColor: AppColors.background, 
      // surface 색상을 background와 동일하게 설정하여 전체 배경이 흰색으로 유지되도록 합니다.
      colorScheme: ColorScheme.fromSeed(
        seedColor: AppColors.primary,
        background: AppColors.background, // Scaffold 배경색
        surface: AppColors.background,    // 카드, 다이얼로그 등 UI 요소의 기본 배경색
      ),
      
      // AppBar 테마
      appBarTheme: const AppBarTheme(
        backgroundColor: AppColors.background,
        foregroundColor: AppColors.textPrimary, // 아이콘 및 뒤로가기 버튼 색상
        elevation: 0,
        titleTextStyle: TextStyle(
          color: AppColors.textPrimary,
          fontSize: 20,
          fontWeight: FontWeight.bold,
        ),
      ),

      // 텍스트 테마
      textTheme: const TextTheme( 
        // 큰 금액, 화면 메인 타이틀 (e.g., '50,000원')
        displaySmall: TextStyle(fontSize: 34, fontWeight: FontWeight.bold, color: AppColors.textPrimary),
        // 화면 서브 타이틀 (e.g., '휘발유 주유를 진행합니다.')
        headlineSmall: TextStyle(fontSize: 24, fontWeight: FontWeight.bold, color: AppColors.textPrimary), 
        // 섹션 타이틀 (e.g., '카드 선택')
        titleLarge: TextStyle(fontSize: 20, fontWeight: FontWeight.bold, color: AppColors.textPrimary),
        // 일반 본문 텍스트
        bodyLarge: TextStyle(fontSize: 18, color: AppColors.textPrimary),
        // 중간 크기 본문 텍스트
        bodyMedium: TextStyle(fontSize: 16, color: AppColors.textPrimary), 
        // 작은 보조 텍스트 (e.g., 주문 ID)
        bodySmall: TextStyle(fontSize: 14, color: AppColors.textSecondary), 
      ),

      // 버튼 테마
      elevatedButtonTheme: ElevatedButtonThemeData(
        style: ElevatedButton.styleFrom(
          backgroundColor: AppColors.primary,
          foregroundColor: Colors.white,
          disabledBackgroundColor: AppColors.surface,
          disabledForegroundColor: AppColors.textSecondary.withOpacity(0.5),
          textStyle: const TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
          padding: const EdgeInsets.symmetric(vertical: 16),
          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
          elevation: 0,
        ),
      ),
    );
  }
}