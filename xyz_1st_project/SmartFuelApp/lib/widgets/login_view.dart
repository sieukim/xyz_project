import 'package:flutter/material.dart';

class LoginView extends StatelessWidget {
  final VoidCallback? onKakaoLoginPressed;
  final VoidCallback? onGoogleLoginPressed;
  final bool isLoading;

  const LoginView({
    super.key,
    this.onKakaoLoginPressed,
    this.onGoogleLoginPressed,
    this.isLoading = false,
  });

  @override
  Widget build(BuildContext context) {
    // Toss-style Design Palette
    const lightGray = Color(0xFFF2F4F6);
    const darkGrayText = Color(0xFF333D4B);

    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        const Spacer(flex: 1),
        Text(
          '환영합니다!',
          style: TextStyle(
            fontSize: 28,
            fontWeight: FontWeight.bold,
            color: darkGrayText,
          ),
        ),
        const SizedBox(height: 8),
        Text(
          'Smart Fuel App에 오신 것을 환영합니다.\nSNS 계정으로 간편하게 시작하세요.',
          style: TextStyle(
            fontSize: 16,
            color: darkGrayText.withOpacity(0.7),
          ),
        ),
        const Spacer(flex: 2),
        _buildLoginButton(
          onPressed: onGoogleLoginPressed,
          iconAsset: 'https://developers.google.com/identity/images/g-logo.png',
          label: 'Google로 계속하기',
          lightGray: lightGray,
          darkGrayText: darkGrayText,
        ),
        const SizedBox(height: 16),
        _buildLoginButton(
          onPressed: onKakaoLoginPressed,
          iconAsset: 'https://www.kakaocorp.com/page/favicon.ico',
          label: '카카오로 계속하기',
          backgroundColor: const Color(0xFFFEE500),
          foregroundColor: Colors.black,
          lightGray: lightGray,
          darkGrayText: darkGrayText,
        ),
        const Spacer(flex: 1),
      ],
    );
  }

  Widget _buildLoginButton({
    required VoidCallback? onPressed,
    required String iconAsset,
    required String label,
    required Color lightGray,
    required Color darkGrayText,
    Color? backgroundColor,
    Color? foregroundColor,
  }) {
    return SizedBox(
      width: double.infinity,
      height: 56,
      child: ElevatedButton(
        onPressed: isLoading ? null : onPressed,
        style: ElevatedButton.styleFrom(
          backgroundColor: backgroundColor ?? lightGray,
          foregroundColor: foregroundColor ?? darkGrayText,
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(12),
          ),
          elevation: 0,
        ),
        child: isLoading
            ? const CircularProgressIndicator()
            : Row(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  Image.network(
                    iconAsset,
                    width: 24,
                    height: 24,
                    errorBuilder: (context, error, stackTrace) {
                      return const Icon(Icons.error);
                    },
                  ),
                  const SizedBox(width: 12),
                  Text(
                    label,
                    style: const TextStyle(
                      fontSize: 16,
                      fontWeight: FontWeight.w600,
                    ),
                  ),
                ],
              ),
      ),
    );
  }
}