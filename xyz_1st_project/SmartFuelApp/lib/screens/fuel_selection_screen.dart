import 'dart:async';
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:flutter_tts/flutter_tts.dart';
import 'package:http/http.dart' as http;
import 'package:speech_to_text/speech_to_text.dart';
import '../services/kakao_login_service.dart';
import 'package:kakao_flutter_sdk_user/kakao_flutter_sdk_user.dart';
import '../services/google_login_service.dart';
import '../services/llm_service.dart';
import 'payment_screen.dart';
import 'login_screen.dart';
import '../widgets/profile_view.dart';

class FuelSelectionScreen extends StatefulWidget {
  const FuelSelectionScreen({Key? key}) : super(key: key);

  @override
  State<FuelSelectionScreen> createState() => _FuelSelectionScreenState();
}

class _FuelSelectionScreenState extends State<FuelSelectionScreen>
    with SingleTickerProviderStateMixin {
  final FlutterTts _flutterTts = FlutterTts();
  final SpeechToText _speechToText = SpeechToText();
  bool _isListening = false;
  String _recognizedWords = '';
  
  String _carNumber = ''; // 상태 변수로 변경
  // OCR HTTP 폴링 관련 변수
  String _ocrText = '차량 번호 인식 대기 중...';
  Timer? _ocrPollingTimer;

  AnimationController? _animationController;

  String fuelType = '휘발유';
  int amount = 50000;
  final int maxAmount = 120000;
  // 프리셋 금액 버튼 목록: 만원 단위로 10,000원 ~ 120,000원
  final List<int> _presets = List.generate(12, (i) => (i + 1) * 10000);
  int? selectedPreset = 50000; // 기본으로 50,000원 선택

  @override
  void initState() {
    super.initState();
    _animationController = AnimationController(
      vsync: this,
      duration: const Duration(seconds: 5),
    );
    // 비동기 초기화 로직을 별도 함수로 분리하여 호출합니다.
    _initialize();
    _startOcrPolling();
  }

  @override
  void dispose() {
    _flutterTts.stop();
    _speechToText.cancel();
    _animationController?.dispose();
    _ocrPollingTimer?.cancel();
    super.dispose();
  }

  /// 화면에 필요한 비동기 서비스들을 초기화합니다.
  Future<void> _initialize() async {
    await _initTts();
    await _initStt();
    // 모든 초기화가 끝난 후, 위젯이 화면에 완전히 그려진 다음 음성 안내를 시작합니다.
    WidgetsBinding.instance.addPostFrameCallback((_) => _speakIntro());
  }

  /// OCR 텍스트를 주기적으로 가져오기 시작합니다.
  void _startOcrPolling() {
    // ❗️ ROS PC의 IP 주소로 변경해주세요.
    const rosIpAddress = '192.168.50.211'; 
    final url = Uri.parse('http://$rosIpAddress:8083/ocr_text');

    _ocrPollingTimer = Timer.periodic(const Duration(seconds: 2), (_) async {
      try {
        final response = await http.get(url).timeout(const Duration(seconds: 2));
        if (response.statusCode == 200) {
          final data = jsonDecode(response.body);
          final newText = data['ocr_text'] as String?;
          if (newText != null && newText.isNotEmpty && newText != _ocrText) {
            setState(() {
              _ocrText = newText;
              _carNumber = newText; // 원본 숫자만 저장
            });
          }
        }
      } catch (e) {
        debugPrint('OCR Polling Error: $e');
        // 네트워크 오류 발생 시 사용자에게 상태를 알립니다.
        setState(() {
          _ocrText = '차량 번호 서버 연결 실패';
        });
      }
    });
  }

  Future<void> _initTts() async {
    await _flutterTts.setLanguage('ko-KR');
    await _flutterTts.setSpeechRate(1.0);
    // speak 함수가 음성 출력을 완료할 때까지 기다리도록 설정합니다.
    await _flutterTts.awaitSpeakCompletion(true);
  }

  Future<void> _initStt() async {
    await _speechToText.initialize(
      onError: (error) => debugPrint('STT Error: $error'),
      onStatus: (status) {
        if (mounted) {
          final isListening = _speechToText.isListening;
          if (_isListening != isListening) {
            setState(() {
              _isListening = isListening;
              if (_isListening) {
                _recognizedWords = '듣는 중...';
                _animationController?.reverse(from: 1.0);
              } else {
                _animationController?.stop();
              }
            });
          }
        }
      },
    );
  }

  Future<void> _speakIntro() async {
    // 숫자와 문자를 분리하여 순차적으로 발화
    // 숫자를 말하는 동안만 발음 속도를 높입니다.
    await _flutterTts.setSpeechRate(1.5);
    for (String num in _carNumber.replaceAll(RegExp(r'[^0-9]'), '').split('')) {
      await _flutterTts.speak(num);
    }
    // 원래 속도로 복원하여 다음 문장을 말합니다.
    await _flutterTts.setSpeechRate(1.0);
    await _flutterTts.speak('고객님, 안녕하세요, 유종과 금액을 말씀해주세요.');
  }

  void _startListening() {
    if (!_speechToText.isAvailable || _isListening) return;
    _speechToText.listen(
      onResult: (result) {
        if (result.finalResult) {
          _processVoiceCommand(result.recognizedWords);
        }
      },
      listenFor: const Duration(seconds: 5),
      localeId: 'ko_KR',
    );
  }

  void _stopListening() {
    if (!_isListening) return;
    _speechToText.stop();
    setState(() {});
  }

  void _toggleListening() {
    if (_isListening) {
      _stopListening();
    } else {
      _startListening();
    }
  }

  /// LLM을 사용하여 음성 명령을 분석하고 상태를 업데이트합니다.
  Future<void> _processVoiceCommand(String command) async {
    if (command.isEmpty) return;
    setState(() {
      _recognizedWords = '분석 중...';
    });

    try {
      final prompt = """
    사용자의 주유 요청에서 유종과 금액을 추출해줘.
    - 유종은 '휘발유', '경유', '전기' 중 하나여야 해.
    - 금액은 만원 단위 숫자(예: 10000, 50000)로 변환해줘.
    - '가득'이라는 표현은 금액을 -1로 설정해줘.
    - 결과는 반드시 JSON 형식으로 반환해줘. 예: {"fuelType": "휘발유", "amount": 50000}
    - 만약 유종이나 금액을 알 수 없다면 null로 설정해줘.

    사용자 요청: "$command"
    """;
      final result = await LlmService.generateContent(prompt);

      setState(() {
        // LLM 응답에서 유종 정보 추출 및 업데이트
        final extractedFuelType = result['fuelType'] as String?;
        if (extractedFuelType != null && ['휘발유', '경유', '전기'].contains(extractedFuelType)) {
          fuelType = extractedFuelType;
        }

        // LLM 응답에서 금액 정보 추출 및 업데이트, 화면 표시용 변수 준비
        String amountForDisplay = '';
        final extractedAmount = result['amount'] as int?;
        if (extractedAmount != null) {
          if (extractedAmount == -1) { // '가득'을 -1로 처리하기로 약속
            amount = maxAmount;
            selectedPreset = maxAmount;
            amountForDisplay = '가득';
          } else if (_presets.contains(extractedAmount)) {
            amount = extractedAmount;
            selectedPreset = extractedAmount;
            amountForDisplay = '${_formatCurrency(extractedAmount)}원';
          }
        }

        // LLM이 수정한 내용을 바탕으로 화면에 표시할 텍스트를 새로 조합
        final correctedFuel = extractedFuelType ?? fuelType;
        _recognizedWords = '$correctedFuel $amountForDisplay'.trim();
      });
    } catch (e) {
      debugPrint('LLM 처리 오류: $e');
      setState(() {
        _recognizedWords = '오류가 발생했습니다. 다시 시도해주세요.';
      });
    }
  }


  @override
  Widget build(BuildContext context) {
    // Toss-style Design Palette
    const tossBlue = Color(0xFF3182F7);
    const lightGray = Color(0xFFF2F4F6);
    const darkGrayText = Color(0xFF333D4B);
    const white = Colors.white;

    return Scaffold(
      backgroundColor: white,
      appBar: AppBar(
        title: const Text('주유 설정', style: TextStyle(color: darkGrayText)),
        backgroundColor: white,
        elevation: 0,
        iconTheme: const IconThemeData(color: darkGrayText),
        actions: [
          // 카카오톡 내 정보 보기 버튼
          IconButton(
            tooltip: '카카오톡 내 정보 보기',
            icon: const Icon(Icons.account_circle_outlined),
            onPressed: () async {
              // 로딩 표시
              if (!mounted) return;
              final navigator = Navigator.of(context);
              showDialog<void>(
                context: context,
                barrierDismissible: false,
                builder: (ctx) =>
                    const Center(child: CircularProgressIndicator()),
              );

              User? user;
              try {
                user = await KakaoLoginService.instance.getUserInfo();
              } catch (e) {
                // close loading dialog using captured navigator
                navigator.pop();
                debugPrint('카카오 사용자 정보 조회 실패: $e');
                if (!mounted) return;
                // schedule dialog on next frame to avoid using the outer BuildContext
                WidgetsBinding.instance.addPostFrameCallback((_) {
                  showDialog<void>(
                    context: context,
                    builder: (ctx) => AlertDialog(
                      title: const Text('오류'),
                      content: const Text('카카오 사용자 정보를 가져오지 못했습니다.'),
                      actions: [
                        TextButton(
                            onPressed: () => Navigator.of(ctx).pop(),
                            child: const Text('확인'))
                      ],
                    ),
                  );
                });
                return;
              }

              // close loading dialog using captured navigator
              navigator.pop();
              if (!mounted) return;

              // 프로필 화면으로 이동
              if (!mounted) return;
              navigator.push(
                MaterialPageRoute(builder: (_) {
                  return Scaffold(
                    appBar: AppBar(title: const Text('내 정보')),
                    body: SafeArea(
                      child: Padding(
                        padding: const EdgeInsets.all(16),
                        child: ProfileView(
                          user: user,
                          loginType: 'kakao',
                          onLogoutPressed: () async {
                            // capture navigator to avoid using context across async gaps
                            final navigator = Navigator.of(context);
                            final should = await showDialog<bool>(
                              context: context,
                              builder: (ctx) => AlertDialog(
                                title: const Text('로그아웃'),
                                content:
                                    const Text('로그아웃하고 로그인 화면으로 이동하시겠습니까?'),
                                actions: [
                                  TextButton(
                                      onPressed: () =>
                                          Navigator.of(ctx).pop(false),
                                      child: const Text('취소')),
                                  TextButton(
                                      onPressed: () =>
                                          Navigator.of(ctx).pop(true),
                                      child: const Text('로그아웃')),
                                ],
                              ),
                            );

                            if (should != true) return;

                            try {
                              await KakaoLoginService.instance.logout();
                            } catch (e) {
                              debugPrint('Kakao logout error: $e');
                            }

                            try {
                              await GoogleLoginService.instance.signOut();
                            } catch (e) {
                              debugPrint('Google signOut error: $e');
                            }

                            if (!mounted) return;

                            navigator.pushAndRemoveUntil(
                              MaterialPageRoute(
                                  builder: (_) => const LoginScreen()),
                              (route) => false,
                            );
                          },
                        ),
                      ),
                    ),
                  );
                }),
              );
            },
          ),

          IconButton(
            tooltip: '로그아웃',
            icon: const Icon(Icons.logout),
            onPressed: () async {
              final navigator = Navigator.of(context);
              final should = await showDialog<bool>(
                context: context,
                builder: (ctx) => AlertDialog(
                  title: const Text('로그아웃'),
                  content: const Text('로그아웃하고 로그인 화면으로 이동하시겠습니까?'),
                  actions: [
                    TextButton(
                        onPressed: () => Navigator.of(ctx).pop(false),
                        child: const Text('취소')),
                    TextButton(
                        onPressed: () => Navigator.of(ctx).pop(true),
                        child: const Text('로그아웃')),
                  ],
                ),
              );

              if (should != true) return;
              if (!mounted) return;

              try {
                await KakaoLoginService.instance.logout();
              } catch (e) {
                debugPrint('Kakao logout error: $e');
              }

              try {
                await GoogleLoginService.instance.signOut();
              } catch (e) {
                debugPrint('Google signOut error: $e');
              }

              if (!mounted) return;

              navigator.pushAndRemoveUntil(
                MaterialPageRoute(builder: (_) => const LoginScreen()),
                (route) => false,
              );
            },
          )
        ],
      ),
      body: SingleChildScrollView(
        padding: const EdgeInsets.all(20),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            // --- 차량 번호 인식 섹션 ---
            _buildSectionTitle('차량 번호', darkGrayText),
            _buildOcrResultCard(lightGray, darkGrayText),
            const SizedBox(height: 32),

            // --- 음성 인식 섹션 ---
            _buildSectionTitle('음성 인식', darkGrayText),
            _buildVoiceCommandCard(lightGray, darkGrayText),
            const SizedBox(height: 32),

            // --- 유종 선택 섹션 ---
            _buildSectionTitle('유종 선택', darkGrayText),
            _buildFuelTypeSelector(tossBlue, lightGray, darkGrayText),
            const SizedBox(height: 32),

            // --- 주유 금액 선택 섹션 ---
            _buildSectionTitle('주유 금액 선택', darkGrayText),
            _buildAmountSelector(tossBlue, lightGray, darkGrayText),
            const SizedBox(height: 40),
          ],
        ),
      ),
      // --- 하단 결제 버튼 ---
      bottomNavigationBar: Padding(
        padding: const EdgeInsets.all(20.0),
        child: ElevatedButton(
          onPressed: selectedPreset == null
              ? null
              : () async {
                  // 결제할 유종과 금액을 음성으로 안내합니다.
                  final String speakAmount =
                      amount == maxAmount ? '가득' : '${_formatCurrency(amount)} 원';
                  
                  // for 루프를 사용하여 차량 번호를 한 글자씩 발음합니다.
                  // 숫자를 말하는 동안만 발음 속도를 높입니다.
                  await _flutterTts.setSpeechRate(1.5);
                  for (String num in _carNumber.replaceAll(RegExp(r'[^0-9]'), '').split('')) {
                    await _flutterTts.speak(num);
                  }
                  // 원래 속도로 복원하여 다음 문장을 말합니다.
                  await _flutterTts.setSpeechRate(1.0);
                  await _flutterTts.speak('고객님, $fuelType, $speakAmount 결제하겠습니다.');

                  if (!mounted) return;
                  
                  Navigator.push(
                    context,
                    MaterialPageRoute(
                      builder: (_) => PaymentScreen(
                        fuelType: fuelType,
                        amount: amount,
                      ),
                    ),
                  );
                },
          style: ElevatedButton.styleFrom(
            backgroundColor: tossBlue,
            foregroundColor: white,
            disabledBackgroundColor: lightGray,
            disabledForegroundColor: darkGrayText.withOpacity(0.38),
            padding: const EdgeInsets.symmetric(vertical: 16),
            shape: RoundedRectangleBorder(
              borderRadius: BorderRadius.circular(12),
            ),
            elevation: 0,
          ),
          child: Text(
            '${_formatCurrency(amount)}원 결제하기',
            style: const TextStyle(fontSize: 18, fontWeight: FontWeight.bold, color: white),
          ),
        ),
      ),
    );
  }

  // 섹션 타이틀 위젯
  Widget _buildSectionTitle(String title, Color color) {
    return Padding(
      padding: const EdgeInsets.only(bottom: 12.0),
      child: Text(title, style: TextStyle(fontSize: 22, fontWeight: FontWeight.bold, color: color)),
    );
  }

  // OCR 결과 표시 위젯
  Widget _buildOcrResultCard(Color bgColor, Color textColor) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 16),
      decoration: BoxDecoration(
        color: bgColor,
        borderRadius: BorderRadius.circular(12),
      ),
      child: Row(
        children: [
          Icon(Icons.directions_car, color: textColor),
          const SizedBox(width: 16),
          Expanded(
            child: Text(
              _ocrText,
              style: TextStyle(fontSize: 18, color: textColor, fontWeight: FontWeight.bold),
            ),
          ),
        ],
      ),
    );
  }
  // 음성 인식 카드 위젯
  Widget _buildVoiceCommandCard(Color bgColor, Color textColor) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
      decoration: BoxDecoration(
        color: bgColor,
        borderRadius: BorderRadius.circular(12),
      ),
      child: Row(
        children: [
          IconButton(
            icon: Icon(_isListening ? Icons.mic_off : Icons.mic, color: textColor),
            onPressed: _toggleListening,
            tooltip: '음성으로 주문',
          ),
          const SizedBox(width: 8),
          Expanded(
            child: Text(
              _recognizedWords.isNotEmpty ? _recognizedWords : '버튼을 누르고 말씀해주세요.',
              style: TextStyle(fontSize: 16, color: textColor),
            ),
          ),
        ],
      ),
    );
  }

  // 유종 선택 위젯
  Widget _buildFuelTypeSelector(Color selectedColor, Color bgColor, Color textColor) {
    final fuelTypes = {
      '휘발유': Icons.local_gas_station,
      '경유': Icons.local_gas_station_outlined,
      '전기': Icons.flash_on,
    };

    return Row(
      children: fuelTypes.entries.map((entry) {
        final type = entry.key;
        final icon = entry.value;
        final isSelected = fuelType == type;

        return Expanded(
          child: GestureDetector(
            onTap: () => setState(() => fuelType = type),
            child: AnimatedContainer(
              duration: const Duration(milliseconds: 200),
              margin: const EdgeInsets.symmetric(horizontal: 4),
              padding: const EdgeInsets.symmetric(vertical: 16),
              decoration: BoxDecoration(
                color: isSelected ? selectedColor : bgColor,
                borderRadius: BorderRadius.circular(12),
              ),
              child: Column(
                children: [
                  Icon(icon, color: isSelected ? Colors.white : textColor, size: 28),
                  const SizedBox(height: 8),
                  Text(type, style: TextStyle(color: isSelected ? Colors.white : textColor, fontWeight: FontWeight.w600)),
                ],
              ),
            ),
          ),
        );
      }).toList(),
    );
  }

  // 금액 선택 위젯
  Widget _buildAmountSelector(Color selectedColor, Color bgColor, Color textColor) {
    return GridView.builder(
      gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(
        crossAxisCount: 3,
        crossAxisSpacing: 8,
        mainAxisSpacing: 8,
        childAspectRatio: 2.5, // 버튼의 가로세로 비율
      ),
      itemCount: _presets.length,
      shrinkWrap: true,
      physics: const NeverScrollableScrollPhysics(),
      itemBuilder: (context, index) {
        final preset = _presets[index];
        final isSelected = selectedPreset == preset;
        final label = preset == maxAmount ? '가득' : '${preset ~/ 10000}만원';

        return GestureDetector(
          onTap: () {
            setState(() {
              selectedPreset = preset;
              amount = preset;
            });
          },
          child: AnimatedContainer(
            duration: const Duration(milliseconds: 200),
            decoration: BoxDecoration(
              color: isSelected ? selectedColor : bgColor,
              borderRadius: BorderRadius.circular(12),
            ),
            child: Center(
              child: Text(
                label,
                style: TextStyle(
                  color: isSelected ? Colors.white : textColor,
                  fontWeight: FontWeight.bold,
                  fontSize: 16,
                ),
              ),
            ),
          ),
        );
      },
    );
  }

  // 간단한 천단위 콤마 포맷터 (의존성 없이 구현)
  String _formatCurrency(int value) {
    final s = value.toString();
    final reg = RegExp(r'\B(?=(\d{3})+(?!\d))');
    return s.replaceAllMapped(reg, (m) => ',');
  }
}