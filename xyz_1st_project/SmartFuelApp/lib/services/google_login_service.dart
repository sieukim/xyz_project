import 'package:google_sign_in/google_sign_in.dart';
import '../config/app_config.dart';

class GoogleLoginService {
  static final GoogleLoginService _instance = GoogleLoginService._internal();
  static GoogleLoginService get instance => _instance;
  GoogleLoginService._internal();

  GoogleSignIn get _googleSignIn => GoogleSignIn(
    clientId: AppConfig.googleSignInClientId,
    scopes: [
      'email',
      'profile',
    ],
  );

  /// 구글 로그인
  Future<GoogleSignInAccount?> signInWithGoogle() async {
    try {
      final GoogleSignInAccount? account = await _googleSignIn.signIn();
      return account;
    } catch (error) {
      // print('구글 로그인 에러: $error');
      return null;
    }
  }

  /// 구글 로그아웃
  Future<void> signOut() async {
    try {
      await _googleSignIn.signOut();
      // print('구글 로그아웃 성공');
    } catch (error) {
      // print('구글 로그아웃 에러: $error');
    }
  }

  /// 현재 로그인된 구글 계정 확인
  GoogleSignInAccount? get currentUser => _googleSignIn.currentUser;

  /// 로그인 상태 확인
  bool get isSignedIn => _googleSignIn.currentUser != null;

  /// 구글 계정 인증 토큰 가져오기
  Future<GoogleSignInAuthentication?> getAuthentication() async {
    try {
      final GoogleSignInAccount? account = _googleSignIn.currentUser;
      if (account != null) {
        return await account.authentication;
      }
      return null;
    } catch (error) {
      // print('구글 인증 토큰 가져오기 에러: $error');
      return null;
    }
  }
}
