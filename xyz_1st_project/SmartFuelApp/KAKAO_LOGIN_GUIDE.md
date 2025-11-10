# 카카오 로그인 설정 가이드

## 1. 카카오 개발자 센터 설정

1. [카카오 디벨로퍼스](https://developers.kakao.com/)에 접속
2. 내 애플리케이션 > 애플리케이션 추가하기
3. 앱 이름, 사업자명 입력 후 저장

## 2. 플랫폼 설정

### Android 설정
1. 플랫폼 > Android 플랫폼 등록
2. 패키지명: `com.example.first_app`
3. 마켓 URL: (선택사항)
4. 키 해시 등록 (필수)

### 키 해시 생성 방법
```bash
# Windows (PowerShell)
keytool -exportcert -alias androiddebugkey -keystore %USERPROFILE%\.android\debug.keystore -storepass android -keypass android | openssl sha1 -binary | openssl base64

# macOS/Linux
keytool -exportcert -alias androiddebugkey -keystore ~/.android/debug.keystore -storepass android -keypass android | openssl sha1 -binary | openssl base64
```

**🔑 현재 생성된 디버그 키 해시 (복사해서 사용하세요):**
```
tIUAb2Nc2OGjU4sFle9LLIkbbDI=
```

> ⚠️ **주의**: 이 키 해시는 개발용(디버그)입니다. 
> 릴리즈 빌드용 키스토어를 사용할 때는 별도의 키 해시가 필요합니다.

## 3. 앱 키 확인 및 적용

1. 카카오 개발자 센터 > 내 애플리케이션 > 앱 설정 > 요약 정보
2. 네이티브 앱 키와 JavaScript 키를 확인

## 4. 코드에 앱 키 적용

`lib/main.dart` 파일에서 다음 부분을 수정:

```dart
KakaoSdk.init(
  nativeAppKey: 'YOUR_NATIVE_APP_KEY', // 여기에 실제 네이티브 앱 키 입력
  javaScriptAppKey: 'YOUR_JAVASCRIPT_APP_KEY', // 여기에 실제 JavaScript 앱 키 입력
);
```

`android/app/src/main/AndroidManifest.xml` 파일에서 다음 부분을 수정:
```xml
<data android:scheme="kakaoYOUR_NATIVE_APP_KEY" />
```
↓
```xml
<data android:scheme="kakao실제네이티브앱키" />
```

## 5. 카카오 로그인 활성화

1. 카카오 개발자 센터 > 제품 설정 > 카카오 로그인
2. 활성화 설정: ON
3. Redirect URI 등록: `kakao{네이티브 앱 키}://oauth`

## 6. 동의항목 설정

1. 카카오 개발자 센터 > 제품 설정 > 카카오 로그인 > 동의항목
2. 필요한 정보 동의항목 설정 (닉네임, 프로필 사진, 이메일 등)

## 주의사항

- 실제 배포 시에는 릴리즈 키 해시도 등록해야 합니다
- 카카오톡이 설치되지 않은 기기에서는 카카오계정 로그인으로 진행됩니다
- 개발 중에는 디버그 키 해시를 사용하고, 배포 시에는 릴리즈 키 해시를 사용해야 합니다