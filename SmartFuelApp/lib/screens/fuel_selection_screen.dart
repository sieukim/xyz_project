import 'package:flutter/material.dart';
import '../services/kakao_login_service.dart';
import 'package:kakao_flutter_sdk_user/kakao_flutter_sdk_user.dart';
import '../services/google_login_service.dart';
import 'payment_screen.dart';
import 'login_screen.dart';
import '../widgets/profile_view.dart';

class FuelSelectionScreen extends StatefulWidget {
  const FuelSelectionScreen({Key? key}) : super(key: key);

  @override
  State<FuelSelectionScreen> createState() => _FuelSelectionScreenState();
}

class _FuelSelectionScreenState extends State<FuelSelectionScreen> {
  String fuelType = '휘발유';
  int amount = 50000;
  final int maxAmount = 150000;
  // 프리셋 금액 버튼 목록: 만원 단위로 10,000원 ~ 150,000원
  final List<int> _presets = List.generate(15, (i) => (i + 1) * 10000);
  int? selectedPreset = 50000; // 기본으로 50,000원 선택

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('주유 설정'),
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
          children: [
            // 유종 선택 박스
            Card(
              shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(12)),
              elevation: 0,
              color: Colors.white,
              child: Padding(
                padding: const EdgeInsets.all(12.0),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    const Text('유종 선택',
                        style: TextStyle(
                            fontSize: 18, fontWeight: FontWeight.w700)),
                    const SizedBox(height: 8),
                    Wrap(
                      spacing: 8,
                      children: ['휘발유', '경유', '전기'].map((type) {
                        final selected = fuelType == type;
                        return ChoiceChip(
                          label: Text(type),
                          selected: selected,
                          onSelected: (sel) {
                            if (!sel) return; // 하나 선택 유지
                            setState(() => fuelType = type);
                          },
                          selectedColor: Theme.of(context).colorScheme.primary,
                          backgroundColor: Colors.grey[100],
                          labelStyle: TextStyle(
                              color: selected ? Colors.white : Colors.black87),
                        );
                      }).toList(),
                    ),
                  ],
                ),
              ),
            ),

            const SizedBox(height: 12),

            // 주유 금액 선택 박스
            Card(
              shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(12)),
              elevation: 0,
              color: Colors.white,
              child: Padding(
                padding: const EdgeInsets.all(12.0),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    const Text('주유 금액 선택',
                        style: TextStyle(
                            fontSize: 18, fontWeight: FontWeight.w700)),
                    const SizedBox(height: 8),
                    Wrap(
                      spacing: 8,
                      runSpacing: 8,
                      children: _presets.map((preset) {
                        final bool isMax = preset == maxAmount;
                        final bool selected = selectedPreset == preset;
                        final label = isMax
                            ? '가득 (${_formatCurrency(preset)}원)'
                            : '${_formatCurrency(preset)}원';
                        return ChoiceChip(
                          label: Text(
                            label,
                            style: TextStyle(
                              fontSize: 14,
                              color: selected ? Colors.white : Colors.black87,
                            ),
                          ),
                          selected: selected,
                          onSelected: (sel) {
                            if (!sel) return; // 항상 하나 선택
                            setState(() {
                              selectedPreset = preset;
                              amount = preset;
                            });
                          },
                          labelPadding: const EdgeInsets.symmetric(
                              horizontal: 12, vertical: 8),
                          selectedColor: Theme.of(context).colorScheme.primary,
                          backgroundColor: Colors.grey[100],
                          shape: RoundedRectangleBorder(
                              borderRadius: BorderRadius.circular(8)),
                        );
                      }).toList(),
                    ),
                  ],
                ),
              ),
            ),

            const SizedBox(height: 12),
            Text(
              '선택 금액: ${amount == maxAmount ? '가득 (${_formatCurrency(amount)}원)' : '${_formatCurrency(amount)}원'}',
              style: const TextStyle(fontSize: 18),
            ),
            const SizedBox(height: 30),
            ElevatedButton.icon(
              onPressed: selectedPreset == null
                  ? null
                  : () {
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
              icon: const Icon(Icons.payment),
              label: const Text('결제하기'),
            )
          ],
        ),
      ),
    );
  }

  // 간단한 천단위 콤마 포맷터 (의존성 없이 구현)
  String _formatCurrency(int value) {
    final s = value.toString();
    final reg = RegExp(r'\B(?=(\d{3})+(?!\d))');
    return s.replaceAllMapped(reg, (m) => ',');
  }
}
