import 'package:flutter/material.dart';
import 'package:kakao_flutter_sdk_user/kakao_flutter_sdk_user.dart';
import 'package:google_sign_in/google_sign_in.dart';
import '../services/kakao_login_service.dart';
import '../services/google_login_service.dart';
import '../widgets/login_view.dart';
import '../widgets/profile_view.dart';
import '../screens/fuel_selection_screen.dart';

class LoginScreen extends StatefulWidget {
  const LoginScreen({super.key});

  @override
  State<LoginScreen> createState() => _LoginScreenState();
}

class _LoginScreenState extends State<LoginScreen> {
  User? _user;
  GoogleSignInAccount? _googleUser;
  bool _isLoggedIn = false;
  bool _isLoading = false;
  String? _loginType; // 'kakao' 또는 'google'

  final KakaoLoginService _kakaoService = KakaoLoginService.instance;
  final GoogleLoginService _googleService = GoogleLoginService.instance;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
    _checkLoginStatus();
  });
  }

  /// 로그인 상태 확인
  Future<void> _checkLoginStatus() async {
    if (_isLoading) return; // ✅ 로그인 시도 중이면 중복 체크 방지

    try {
      bool isKakaoLoggedIn = await _kakaoService.isLoggedIn();
      if (isKakaoLoggedIn) {
        await _getKakaoUserInfo();
        WidgetsBinding.instance.addPostFrameCallback((_) {
          if (!mounted) return;
          Navigator.pushReplacement(
            context,
            MaterialPageRoute(builder: (_) => const FuelSelectionScreen()),
          );
        });
        return;
      }

      if (_googleService.isSignedIn) {
        await _getGoogleUserInfo();
        return;
      }
    } catch (e) {
      debugPrint('로그인 상태 확인 실패: $e');
    }
  }

  /// 구글 로그인 실행
  Future<void> _loginWithGoogle() async {
    setState(() {
      _isLoading = true;
    });

    try {
      final GoogleSignInAccount? account =
          await _googleService.signInWithGoogle();

      if (account != null) {
        setState(() {
          _googleUser = account;
          _isLoggedIn = true;
          _loginType = 'google';
        });
        // print('구글 로그인 성공: ${account.displayName}');
        _showSuccessSnackBar('구글 로그인 성공!');
        // 구글 로그인 성공 시 주유 선택 화면으로 이동 (post-frame으로 안전하게 실행)
        WidgetsBinding.instance.addPostFrameCallback((_) {
          if (!mounted) return;
          Navigator.pushReplacement(
            context,
            MaterialPageRoute(builder: (_) => const FuelSelectionScreen()),
          );
        });
      } else {
        // print('구글 로그인이 취소되었습니다.');
        _showErrorSnackBar('구글 로그인이 취소되었습니다.');
      }
    } catch (error) {
      // print('구글 로그인 실패: $error');
      _showErrorSnackBar('구글 로그인에 실패했습니다. 다시 시도해주세요.');
    } finally {
      setState(() {
        _isLoading = false;
      });
    }
  }

  /// 카카오 로그인 실행
  Future<void> _loginWithKakao() async {
    if (_isLoading) return;

    setState(() => _isLoading = true);
    debugPrint('🚀 카카오 로그인 시도');

    try {
      final alreadyLoggedIn = await _kakaoService.isLoggedIn();
      if (alreadyLoggedIn) {
        debugPrint('✅ 이미 로그인된 세션 감지 → 다음 화면으로 이동');
        _navigateToFuelSelection();
        return;
      }

      OAuthToken? token = await _kakaoService.loginWithKakaoTalk();
      debugPrint('🟢 loginWithKakaoTalk 결과: ${token != null ? "성공" : "null"}');

      final user = await UserApi.instance.me();
      debugPrint('👤 로그인 성공: ${user.kakaoAccount?.profile?.nickname}');
      await _getKakaoUserInfo();
      _showSuccessSnackBar('로그인 성공!');

      // ✅ 로그인 종료 전에 화면 전환 먼저
      _navigateToFuelSelection();

    } catch (e) {
      debugPrint('❌ 로그인 중 예외: $e');
      _showErrorSnackBar('로그인 실패: $e');
    } finally {
      // ✅ 네비게이션 완료 후 로딩 해제
      Future.delayed(const Duration(milliseconds: 500), () {
        if (mounted) setState(() => _isLoading = false);
      });
    }
  }
  
  /// 사용자 정보 가져오기
  Future<void> _getKakaoUserInfo() async {
    try {
      User kakaoUser = await UserApi.instance.me();
      setState(() {
        _user = kakaoUser;
        //_isLoggedIn = true;
      });
      // print('사용자 정보 가져오기 성공');
    } catch (error) {
      // print('사용자 정보 가져오기 실패: $error');
    }
  }

  Future<void> _getGoogleUserInfo() async {
    try {
      final GoogleSignInAccount? googleUser =
          GoogleLoginService.instance.currentUser;
      if (googleUser != null) {
        setState(() {
          _googleUser = googleUser;
          _isLoggedIn = true;
        });
        // print('구글 사용자 정보 가져오기 성공: ${googleUser.displayName}');
      }
    } catch (error) {
      // print('구글 사용자 정보 가져오기 실패: $error');
    }
  }

  /// 로그아웃
  Future<void> _logout() async {
    try {
      if (_loginType == 'kakao') {
        await _kakaoService.logout();
        setState(() {
          _user = null;
          _loginType = null;
          _isLoggedIn = false;
        });
      } else if (_loginType == 'google') {
        await GoogleLoginService.instance.signOut();
        setState(() {
          _googleUser = null;
          _loginType = null;
          _isLoggedIn = false;
        });
      }
      _showSuccessSnackBar('로그아웃되었습니다.');
    } catch (error) {
      // print('로그아웃 실패: $error');
      _showErrorSnackBar('로그아웃에 실패했습니다.');
    }
  }

  void _navigateToFuelSelection() {
    if (!mounted) return;
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (!mounted) return;
      Navigator.of(context).pushAndRemoveUntil(
        MaterialPageRoute(builder: (_) => const FuelSelectionScreen()),
        (route) => false,
      );
    });
  }

  /// 성공 스낵바 표시
  void _showSuccessSnackBar(String message) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: Colors.green,
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
      ),
    );
  }

  /// 에러 스낵바 표시
  void _showErrorSnackBar(String message) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: Colors.red,
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('로그인'),
      ),
      body: SafeArea(
        child: Padding(
          padding: const EdgeInsets.all(24.0),
          child: LoginView(
                  onKakaoLoginPressed: _loginWithKakao,
                  onGoogleLoginPressed: _loginWithGoogle,
                  isLoading: _isLoading,
                ),
        ),
      ),
    );
  }
}
