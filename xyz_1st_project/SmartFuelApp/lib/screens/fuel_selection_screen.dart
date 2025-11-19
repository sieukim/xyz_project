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
import '../theme/app_theme.dart';

/// 유종과 금액을 선택하는 주유 설정 화면
class FuelSelectionScreen extends StatefulWidget {
  const FuelSelectionScreen({Key? key}) : super(key: key);

  @override
  State<FuelSelectionScreen> createState() => _FuelSelectionScreenStateWrapper();
}

/// VoiceInteractionService를 하위 위젯에 제공하기 위한 래퍼
class _FuelSelectionScreenStateWrapper extends State<FuelSelectionScreen> {
  @override
  Widget build(BuildContext context) {
    return ChangeNotifierProvider(
      create: (_) => VoiceInteractionService(),
      child: const _FuelSelectionScreenContent(),
    );
  }
}

/// 음성 대화의 현재 맥락을 관리
enum ConversationContext {
  none, 
  awaitingFinalConfirmation, 
}

/// 주유 설정 화면의 실제 UI와 상태를 포함하는 위젯
class _FuelSelectionScreenContent extends StatefulWidget {
  const _FuelSelectionScreenContent({Key? key}) : super(key: key);

  @override
  State<_FuelSelectionScreenContent> createState() => _FuelSelectionScreenState();
}

/// 주유 설정 화면의 상태 관리 로직 (차량 번호 인식, 음성 명령 처리, UI 업데이트)
class _FuelSelectionScreenState extends State<_FuelSelectionScreenContent>
    with SingleTickerProviderStateMixin {
  String _carNumber = ''; 
  String _ocrText = '차량 번호 인식 대기 중...';
  Timer? _ocrPollingTimer;
  AnimationController? _animationController;
  String fuelType = '휘발유';
  int amount = 50000;
  final int maxAmount = 120000;
  final List<int> _presets = List.generate(12, (i) => (i + 1) * 10000);
  int? selectedPreset = 50000; 
  ConversationContext _conversationContext = ConversationContext.none;
  late VoiceInteractionService _voiceService;

  /// 위젯 초기화
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

  /// 위젯 종료 시 리소스 정리
  @override
  void dispose() {
    _animationController?.dispose();
    _ocrPollingTimer?.cancel();
    super.dispose();
  }

  /// 화면에 필요한 서비스 초기화
  Future<void> _initialize() async {
    _voiceService.onResult = _processVoiceCommand;
    _startOcrPolling();
    WidgetsBinding.instance.addPostFrameCallback((_) => _speakIntroAfterOcr());
  }

  /// 차량 번호 OCR 서버로부터 주기적으로 텍스트 조회
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
              _carNumber = newText; 
            });
          }
        }
      } catch (e) {
        setState(() {
          _ocrText = '차량 번호 서버 연결 실패';
        });
      }
    });
  }

  /// 차량 번호 인식 후 초기 음성 안내 시작
  Future<void> _speakIntroAfterOcr() async {
    int attempts = 0;
    while (_carNumber.isEmpty && attempts < 5) {
      await Future.delayed(const Duration(seconds: 1));
      attempts++;
    }

    if (!mounted || _carNumber.isEmpty) return;

    final followUpMessage = '고객님, 안녕하세요. 유종과 금액을 말씀해주세요.';
    _voiceService.speakCharacterByCharacter(_carNumber, followUpMessage);
  }

  /// 사용자 음성 명령 처리
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

    _handleConversationAction(action);
  }

  /// 대화 분석 결과에 따른 액션 수행
  void _handleConversationAction(ConversationAction action) {
    if (!mounted) return;

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

    if (action.speakText != null) {
      if (action.shouldListenNext) {
        _voiceService.speakAndListen(action.speakText!);
      } else {
        _voiceService.speak(action.speakText!);
      }
    }
  }

  /// 결제 화면으로 이동
  void _navigateToPayment() {
    if (!mounted) return;
    Navigator.pushReplacement(
      context,
      MaterialPageRoute(
        builder: (_) => PaymentScreen(fuelType: fuelType, amount: amount),
      ),
    );
  }

  /// 로그아웃 처리
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

  /// 화면 UI 구성
  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);

    return Scaffold(
      appBar: AppBar(
        title: const Text('주유 설정'),
        iconTheme: IconThemeData(color: Theme.of(context).appBarTheme.foregroundColor),
        actionsIconTheme: IconThemeData(color: Theme.of(context).appBarTheme.foregroundColor),
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
            _buildSectionTitle('차량 번호'),
            _buildOcrResultCard(),
            const SizedBox(height: 32),

            _buildSectionTitle('음성 인식'),
            _buildVoiceCommandCard(),
            const SizedBox(height: 32),

            _buildSectionTitle('유종 선택'),
            _buildFuelTypeSelector(),
            const SizedBox(height: 32),

            _buildSectionTitle('주유 금액 선택'),
            _buildAmountSelector(),
            const SizedBox(height: 40),
          ],
        ),
      ),

      bottomNavigationBar: Padding(
        padding: const EdgeInsets.all(20.0),
        child: Consumer<VoiceInteractionService>(
          builder: (context, voiceService, child) => ElevatedButton(onPressed: (selectedPreset == null || voiceService.isProcessing)
              ? null
              : () async {
                  final String speakAmount =
                      amount == maxAmount ? '가득' : '${_formatCurrency(amount)} 원';

                  await voiceService.speak('$fuelType, $speakAmount 결제하겠습니다.');
                  if (!mounted) return;
                  _navigateToPayment();
                },
            style: theme.elevatedButtonTheme.style,
            child: const Text('결제하기'),
          ),
        ),
      ),
    );
  }

  /// 섹션 제목 위젯 생성
  Widget _buildSectionTitle(String title) {
    return Padding(
      padding: const EdgeInsets.only(bottom: 12.0),
      child: Text(title, style: Theme.of(context).textTheme.titleLarge),
    );
  }

  /// 차량 번호 표시 카드 위젯 생성
  Widget _buildOcrResultCard() {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 16),
      decoration: BoxDecoration(
        color: AppColors.surface, 
        borderRadius: BorderRadius.circular(12),
      ),
      child: Row(
        children: [
          const Icon(Icons.directions_car, color: AppColors.textPrimary),
          const SizedBox(width: 16),
          Expanded(
            child: Text(
              _ocrText,
              style: Theme.of(context).textTheme.bodyMedium?.copyWith(fontWeight: FontWeight.bold),
            ),
          ),
        ],
      ),
    );
  }
  
  /// 음성 명령 상태 표시 바 위젯 생성
  Widget _buildVoiceCommandCard() {
    return VoiceCommandBar(
      initialText: '음성으로 주유 설정을 해보세요.',
      backgroundColor: AppColors.surface,
      textColor: AppColors.textPrimary,
    );
  }

  /// 유종 선택 버튼 그룹 위젯 생성
  Widget _buildFuelTypeSelector() {
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
            onTap: Provider.of<VoiceInteractionService>(context).isProcessing
                ? null
                : () {
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
                color: isSelected ? AppColors.primary : AppColors.surface, 
                borderRadius: BorderRadius.circular(12),
              ),
              child: Column(
                children: [
                  Icon(icon, color: isSelected ? Colors.white : AppColors.textPrimary, size: 28),
                  const SizedBox(height: 8),
                  Text(type, style: TextStyle(color: isSelected ? Colors.white : AppColors.textPrimary, fontWeight: FontWeight.w600)),
                ],
              ),
            ),
          ),
        );
      }).toList(),
    );
  }
  
  /// 주유 금액 선택 버튼 그리드 위젯 생성
  Widget _buildAmountSelector() {
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
          onTap: Provider.of<VoiceInteractionService>(context).isProcessing
              ? null
              : () {
                  setState(() {
                    selectedPreset = preset;
                    amount = preset;
                    Provider.of<VoiceInteractionService>(context, listen: false).deactivateOnManualSelection();
                  });
                },
          child: AnimatedContainer(
            duration: const Duration(milliseconds: 200),
            decoration: BoxDecoration(
              color: isSelected ? AppColors.primary : AppColors.surface, 
              borderRadius: BorderRadius.circular(12),
            ),
            child: Center(
              child: Text(
                label,
                style: TextStyle(
                  color: isSelected ? Colors.white : AppColors.textPrimary,
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

  /// 숫자를 통화 형식(,)으로 변환
  String _formatCurrency(int value) {
    final s = value.toString();
    final reg = RegExp(r'\B(?=(\d{3})+(?!\d))');
    return s.replaceAllMapped(reg, (m) => ',');
  }
}