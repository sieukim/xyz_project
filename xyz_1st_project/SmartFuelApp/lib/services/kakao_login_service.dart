import 'package:flutter/services.dart';
import 'package:kakao_flutter_sdk_user/kakao_flutter_sdk_user.dart';

class KakaoLoginService {
  static KakaoLoginService? _instance;
  static KakaoLoginService get instance => _instance ??= KakaoLoginService._();

  KakaoLoginService._();

  /// 카카오톡으로 로그인
  Future<OAuthToken?> loginWithKakaoTalk() async {
    try {
      bool isInstalled = await isKakaoTalkInstalled();

      if (isInstalled) {
        return await UserApi.instance.loginWithKakaoTalk();
      } else {
        return await loginWithKakaoAccount();
      }
    } catch (error) {
      // print('카카오톡으로 로그인 실패: $error');

      // 사용자가 카카오톡 설치 후 디바이스 권한 요청 화면에서 로그인을 취소한 경우,
      // 의도적인 로그인 취소로 보고 카카오계정으로 로그인 시도 없이 로그인 취소로 처리
      if (error is PlatformException && error.code == 'CANCELED') {
        return null;
      }

      // 카카오톡에 연결된 카카오계정이 없는 경우, 카카오계정으로 로그인
      try {
        return await UserApi.instance.loginWithKakaoAccount();
      } catch (error) {
        // print('카카오계정으로 로그인 실패: $error');
        rethrow;
      }
    }
  }

  /// 카카오 계정으로 로그인
  Future<OAuthToken?> loginWithKakaoAccount() async {
    try {
      return await UserApi.instance.loginWithKakaoAccount();
    } catch (error) {
      // print('카카오계정으로 로그인 실패: $error');
      rethrow;
    }
  }

  /// 사용자 정보 가져오기
  Future<User> getUserInfo() async {
    try {
      return await UserApi.instance.me();
    } catch (error) {
      // print('사용자 정보 요청 실패: $error');
      rethrow;
    }
  }

  /// 로그아웃
  Future<void> logout() async {
    try {
      await UserApi.instance.logout();
      // print('로그아웃 성공, SDK에서 토큰 삭제');
    } catch (error) {
      // print('로그아웃 실패, SDK에서 토큰 삭제: $error');
      rethrow;
    }
  }

  /// 연결 끊기 (회원 탈퇴)
  Future<void> unlink() async {
    try {
      await UserApi.instance.unlink();
      // print('연결 끊기 성공, SDK에서 토큰 삭제');
    } catch (error) {
      // print('연결 끊기 실패: $error');
      rethrow;
    }
  }

  /// 토큰 정보 확인
  Future<AccessTokenInfo> getAccessTokenInfo() async {
    try {
      return await UserApi.instance.accessTokenInfo();
    } catch (error) {
      // print('토큰 정보 확인 실패: $error');
      rethrow;
    }
  }

  /// 로그인 상태 확인
  Future<bool> isLoggedIn() async {
    try {
    final tokenInfo = await UserApi.instance.accessTokenInfo();
    return tokenInfo.id != null;
  } catch (_) {
    return false;
  }
  }
}
