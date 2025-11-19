import 'dart:async';
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'package:provider/provider.dart';
import '../services/kakao_login_service.dart';
import 'package:kakao_flutter_sdk_user/kakao_flutter_sdk_user.dart';
import '../services/google_login_service.dart';
import '../services/conversation_manager_service.dart';
import '../config/app_config.dart';
import '../services/llm_service.dart';
import 'payment_screen.dart';
import 'login_screen.dart';
import '../widgets/voice_command_bar.dart';
import '../widgets/profile_view.dart';

import '../services/voice_interaction_service.dart';
class FuelSelectionScreen extends StatefulWidget {
  const FuelSelectionScreen({Key? key}) : super(key: key);

  @override
  State<FuelSelectionScreen> createState() => _FuelSelectionScreenStateWrapper();
}

/// ChangeNotifierProvider를 사용하기 위한 Wrapper 클래스
class _FuelSelectionScreenStateWrapper extends State<FuelSelectionScreen> {
  @override
  Widget build(BuildContext context) {
    return ChangeNotifierProvider(
      create: (_) => VoiceInteractionService(),
      child: const _FuelSelectionScreenContent(),
    );
  }
}

/// 대화의 현재 상태를 관리하기 위한 열거형
enum ConversationContext {
  none, // 초기 상태 또는 정보 수집 완료
  awaitingFinalConfirmation, // 최종 결제 확인 대기 ("~로 결제할까요?")
}

class _FuelSelectionScreenContent extends StatefulWidget {
  const _FuelSelectionScreenContent({Key? key}) : super(key: key);

  @override
  State<_FuelSelectionScreenContent> createState() => _FuelSelectionScreenState();
}

class _FuelSelectionScreenState extends State<_FuelSelectionScreenContent>
    with SingleTickerProviderStateMixin {
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

  // 대화 상태를 추적하는 변수
  ConversationContext _conversationContext = ConversationContext.none;
  late VoiceInteractionService _voiceService;

  @override
  void initState() {
    super.initState();
    _voiceService = Provider.of<VoiceInteractionService>(context, listen: false);
    _animationController = AnimationController(
      vsync: this,
      duration: const Duration(seconds: 2),
    );
    _initialize();
  }

  @override
  void dispose() {
    _animationController?.dispose();
    _ocrPollingTimer?.cancel();
    super.dispose();
  }

  /// 화면에 필요한 비동기 서비스들을 초기화합니다.
  Future<void> _initialize() async {
    _voiceService.onResult = _processVoiceCommand;
    _startOcrPolling();
    // 위젯이 완전히 그려진 후, 초기 음성 안내 및 대화 시작
    WidgetsBinding.instance.addPostFrameCallback((_) => _speakIntroAfterOcr());
  }

  /// OCR 텍스트를 주기적으로 가져오기 시작합니다.
  void _startOcrPolling() {
    final url = Uri.parse('${AppConfig.ocrStreamerUrl}/ocr_text');

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

  /// OCR로 차량 번호가 인식된 후 초기 음성 안내를 합니다.
  Future<void> _speakIntroAfterOcr() async {
    // 차량 번호가 인식될 때까지 잠시 대기합니다. (최대 5초)
    int attempts = 0;
    while (_carNumber.isEmpty && attempts < 5) {
      await Future.delayed(const Duration(seconds: 1));
      attempts++;
    }

    if (!mounted || _carNumber.isEmpty) return;

    final introMessage = '$_carNumber 고객님, 안녕하세요. 유종과 금액을 말씀해주세요.';
    _voiceService.speakAndListen(introMessage);
  }

  /// LLM을 사용하여 음성 명령을 분석하고 상태를 업데이트합니다.
  Future<void> _processVoiceCommand(String command) async {
    if (command.isEmpty) return;
    final manager = ConversationManagerService();
    final action = await manager.processVoiceCommand(
      command: command,
      screen: ConversationScreen.fuelSelection,
      context: {
        'fuelType': fuelType,
        'amount': amount,
        'conversationContext': _conversationContext.name,
      },
    );

    // UI 업데이트 및 다음 행동 처리
    _handleConversationAction(action);
  }

  void _handleConversationAction(ConversationAction action) {
    if (!mounted) return;

    // 1. 상태 업데이트
    final updates = action.stateUpdate;
    if (updates.isNotEmpty) {
      setState(() {
        if (updates['deactivate_voice'] == true) {
          _voiceService.deactivateFeature(action.speakText ?? "네, 직접 선택해주세요.");
        }
        if (updates['navigate_to_payment'] == true) {
          _navigateToPayment();
        }
        if (updates['reset_context'] == true) {
          _conversationContext = ConversationContext.none;
        }
        if (updates['set_context'] != null) {
          _conversationContext = ConversationContext.awaitingFinalConfirmation;
        }
        if (updates['fuelType'] != null) {
          fuelType = updates['fuelType'];
        }
        if (updates['amount'] != null) {
          final newAmount = updates['amount'] as int;
          if (newAmount == -1) {
            amount = maxAmount;
            selectedPreset = maxAmount;
          } else {
            amount = newAmount;
            selectedPreset = newAmount;
          }
        }
      });
    }

    // 2. 음성 안내 및 다음 듣기
    if (action.speakText != null) {
      if (action.shouldListenNext) {
        _voiceService.speakAndListen(action.speakText!);
      } else {
        _voiceService.speak(action.speakText!);
      }
    }
  }

  /// 결제 화면으로 이동하는 함수
  void _navigateToPayment() {
    if (!mounted) return;
    Navigator.pushReplacement(
      context,
      MaterialPageRoute(
        builder: (_) => PaymentScreen(fuelType: fuelType, amount: amount),
      ),
    );
  }

  /// 공통 로그아웃 로직
  Future<void> _handleLogout() async {
    final navigator = Navigator.of(context);
    final shouldLogout = await showDialog<bool>(
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

    if (shouldLogout != true || !mounted) return;

    // 모든 소셜 로그아웃 시도
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
  }

  @override
  Widget build(BuildContext context) {
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
          IconButton(
            tooltip: '카카오톡 내 정보 보기',
            icon: const Icon(Icons.account_circle_outlined),
            onPressed: () async {
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
                navigator.pop();
                debugPrint('카카오 사용자 정보 조회 실패: $e');
                if (!mounted) return;
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

              navigator.pop();
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
                          onLogoutPressed: _handleLogout,
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
            onPressed: _handleLogout,
          )
        ],
      ),
      body: SingleChildScrollView(
        padding: const EdgeInsets.all(20),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            _buildSectionTitle('차량 번호', darkGrayText),
            _buildOcrResultCard(lightGray, darkGrayText),
            const SizedBox(height: 32),

            _buildSectionTitle('음성 인식', darkGrayText),
            _buildVoiceCommandCard(lightGray, darkGrayText),
            const SizedBox(height: 32),

            _buildSectionTitle('유종 선택', darkGrayText),
            _buildFuelTypeSelector(tossBlue, lightGray, darkGrayText),
            const SizedBox(height: 32),

            _buildSectionTitle('주유 금액 선택', darkGrayText),
            _buildAmountSelector(tossBlue, lightGray, darkGrayText),
            const SizedBox(height: 40),
          ],
        ),
      ),

      bottomNavigationBar: Padding(
        padding: const EdgeInsets.all(20.0),
        child: Consumer<VoiceInteractionService>(
          builder: (context, voiceService, child) => ElevatedButton(
          onPressed: (selectedPreset == null || voiceService.isProcessing)
              ? null
              : () async {
                  final String speakAmount =
                      amount == maxAmount ? '가득' : '${_formatCurrency(amount)} 원';

                  await voiceService.speak('$fuelType, $speakAmount 결제하겠습니다.');
                  if (!mounted) return;
                  _navigateToPayment();
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
          child: const Text(
            '결제하기',
            style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
          ),
        ),
        ),
      ),
    );
  }

  Widget _buildSectionTitle(String title, Color color) {
    return Padding(
      padding: const EdgeInsets.only(bottom: 12.0),
      child: Text(title, style: TextStyle(fontSize: 22, fontWeight: FontWeight.bold, color: color)),
    );
  }

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

  Widget _buildVoiceCommandCard(Color bgColor, Color textColor) {
    final voiceService = Provider.of<VoiceInteractionService>(context, listen: false);
    return VoiceCommandBar(
      initialText: '음성으로 주유 설정을 해보세요.',
      backgroundColor: bgColor,
      textColor: textColor,
    );
  }

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
            onTap: Provider.of<VoiceInteractionService>(context).isProcessing ? null : () {
              setState(() {
                fuelType = type;
                Provider.of<VoiceInteractionService>(context, listen: false).deactivateOnManualSelection();
              });
            },
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

  Widget _buildAmountSelector(Color selectedColor, Color bgColor, Color textColor) {
    return GridView.builder(
      gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(
        crossAxisCount: 3,
        crossAxisSpacing: 8,
        mainAxisSpacing: 8,
        childAspectRatio: 2.5,
      ),
      itemCount: _presets.length,
      shrinkWrap: true,
      physics: const NeverScrollableScrollPhysics(),
      itemBuilder: (context, index) {
        final preset = _presets[index];
        final isSelected = selectedPreset == preset;
        final label = preset == maxAmount ? '가득' : '${preset ~/ 10000}만원';

        return GestureDetector(
          onTap: Provider.of<VoiceInteractionService>(context).isProcessing ? null : () {
            setState(() {
              selectedPreset = preset;
              amount = preset;
              Provider.of<VoiceInteractionService>(context, listen: false).deactivateOnManualSelection();
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

  String _formatCurrency(int value) {
    final s = value.toString();
    final reg = RegExp(r'\B(?=(\d{3})+(?!\d))');
    return s.replaceAllMapped(reg, (m) => ',');
  }
}