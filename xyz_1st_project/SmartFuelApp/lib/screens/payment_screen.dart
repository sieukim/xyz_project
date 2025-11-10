import 'package:flutter/material.dart';
import 'dart:convert';
import 'package:http/http.dart' as http;
import 'package:flutter_tts/flutter_tts.dart';
import 'fuel_progress_screen.dart';
import 'dart:math';

class PaymentScreen extends StatefulWidget {
  final String fuelType;
  final int amount;

  const PaymentScreen({Key? key, required this.fuelType, required this.amount})
      : super(key: key);

  @override
  State<PaymentScreen> createState() => _PaymentScreenState();
}

class _PaymentScreenState extends State<PaymentScreen> {
  final FlutterTts _flutterTts = FlutterTts();
  bool isProcessing = false;
  final String rosServerUrl = 'http://192.168.50.210:8000/start_fuel'; // 수정 필요
  String _selectedPaymentMethod = '신용카드';
  int _selectedCardIndex = 0;
  final List<String> _cardNumbers = [];

  @override
  void initState() {
    super.initState();
    _initTts();
    // Generate random card numbers
    for (int i = 0; i < 5; i++) {
      _cardNumbers.add((Random().nextInt(9000) + 1000).toString());
    }
  }

  Future<void> _initTts() async {
    await _flutterTts.setLanguage('ko-KR');
    await _flutterTts.setSpeechRate(1.0);
  }

  Future<void> simulatePayment() async {
    setState(() => isProcessing = true);

    await Future.delayed(const Duration(seconds: 2)); // 가상 결제 지연

    try {
      final orderId =
          '${DateTime.now().millisecondsSinceEpoch}-${Random().nextInt(900000)}';
      final payload = {
        'event': 'payment_complete',
        'orderId': orderId,
        'fuelType': widget.fuelType,
        'amount': widget.amount,
        'paymentMethod': _selectedPaymentMethod,
        'source': 'mobile_app',
      };

      final res = await http
          .post(
            Uri.parse(rosServerUrl),
            headers: {'Content-Type': 'application/json'},
            body: jsonEncode(payload),
          )
          .timeout(const Duration(seconds: 8));

      if (res.statusCode == 200) {
        await _flutterTts.speak("결제를 완료했습니다.");

        String returnedOrderId = orderId;
        try {
          final body = jsonDecode(res.body);
          if (body is Map && body['orderId'] != null) {
            returnedOrderId = body['orderId'].toString();
          }
        } catch (_) {}

        if (!mounted) return;
        Navigator.pushReplacement(
          context,
          MaterialPageRoute(
            builder: (_) => FuelProgressScreen(
              orderId: returnedOrderId,
              rosBaseUrl:
                  rosServerUrl.replaceFirst(RegExp(r'/start_fuel\/?'), ''),
            ),
          ),
        );
      } else {
        throw Exception('서버 오류 (${res.statusCode})');
      }
    } catch (e) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('결제 실패: $e')),
      );
    } finally {
      setState(() => isProcessing = false);
    }
  }

  Widget _buildCardItem(int index) {
    final isSelected = _selectedCardIndex == index;
    final cardGradients = [
      [const Color(0xFF6B7684), const Color(0xFF333D4B)], // Grey
      [const Color(0xFF0052D4), const Color(0xFF4364F7), const Color(0xFF6FB1FC)], // Blue
      [const Color(0xFFD4145A), const Color(0xFFFBB03B)], // Orange/Pink
      [const Color(0xFF009245), const Color(0xFFFCEE21)], // Green/Yellow
      [const Color(0xFF4776E6), const Color(0xFF8E54E9)], // Purple/Blue
    ];

    return Transform.scale(
      scale: isSelected ? 1.0 : 0.95,
      child: Container(
        width: 200,
        margin: const EdgeInsets.symmetric(horizontal: 8),
        decoration: BoxDecoration(
          gradient: LinearGradient(
            colors: cardGradients[index % cardGradients.length],
            begin: Alignment.topLeft,
            end: Alignment.bottomRight,
          ),
          borderRadius: BorderRadius.circular(12),
          border: isSelected ? Border.all(color: const Color(0xFF3182F7), width: 3) : null,
          boxShadow: [
            BoxShadow(
              color: Colors.black.withOpacity(0.2),
              blurRadius: 8,
              offset: const Offset(0, 4),
            ),
          ],
        ),
        child: Padding(
          padding: const EdgeInsets.all(16.0),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text(
                'My Card ${index + 1}',
                style: const TextStyle(color: Colors.white, fontSize: 16, fontWeight: FontWeight.bold),
              ),
              const Spacer(),
              Text(
                '**** **** **** ${_cardNumbers[index]}',
                style: const TextStyle(color: Colors.white70, fontSize: 14, letterSpacing: 1.5),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildCardList() {
    return SizedBox(
      height: 150,
      child: ListView.builder(
        scrollDirection: Axis.horizontal,
        itemCount: 5,
        itemBuilder: (context, index) {
          return GestureDetector(
            onTap: () {
              setState(() {
                _selectedCardIndex = index;
                _selectedPaymentMethod = '신용카드';
              });
            },
            child: _buildCardItem(index),
          );
        },
      ),
    );
  }

  Widget _buildPayButtons() {
    final payMethods = {
      '네이버페이': {'bg': const Color(0xFFE6F8F0), 'text': const Color(0xFF03C75A)},
      '카카오페이': {'bg': const Color(0xFFFFFBE6), 'text': const Color(0xFF3C1E1E)},
      '토스페이': {'bg': const Color(0xFFE6F0FF), 'text': const Color(0xFF0064FF)},
    };

    return Row(
      children: payMethods.entries.map((entry) {
        final methodName = entry.key;
        final bgColor = entry.value['bg']!;
        final textColor = entry.value['text']!;
        final isSelected = _selectedPaymentMethod == methodName;

        return Expanded(
          child: GestureDetector(
            onTap: () {
              setState(() {
                _selectedPaymentMethod = methodName;
                _selectedCardIndex = -1; // Deselect card
              });
            },
            child: AnimatedContainer(
              duration: const Duration(milliseconds: 200),
              height: 60,
              margin: const EdgeInsets.symmetric(horizontal: 4),
              decoration: BoxDecoration(
                color: bgColor,
                borderRadius: BorderRadius.circular(12),
                border: isSelected ? Border.all(color: const Color(0xFF3182F7), width: 3) : null,
              ),
              child: Center(
                child: Text(
                  methodName,
                  style: TextStyle(
                    fontSize: 16,
                    fontWeight: FontWeight.bold,
                    color: textColor,
                  ),
                ), 
              ),
            ),
          ),
        );
      }).toList(),
    );
  }

  @override
  Widget build(BuildContext context) {
    const tossBlue = Color(0xFF3182F7);
    const darkGrayText = Color(0xFF333D4B);
    const lightGrayText = Color(0xFF6B7684);
    const white = Colors.white;

    return Scaffold(
      backgroundColor: white,
      appBar: AppBar(
        title: const Text('결제', style: TextStyle(color: darkGrayText)),
        backgroundColor: white,
        elevation: 0,
        iconTheme: const IconThemeData(color: darkGrayText),
      ),
      body: isProcessing
          ? Center(
              child: CircularProgressIndicator(
                color: tossBlue,
              ),
            )
          : SingleChildScrollView(
              padding: const EdgeInsets.all(24.0),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    '${widget.amount}원',
                    style: const TextStyle(
                      fontSize: 32,
                      fontWeight: FontWeight.bold,
                      color: darkGrayText,
                    ),
                  ),
                  const SizedBox(height: 8),
                  Text(
                    '${widget.fuelType} 주유를 진행합니다.',
                    style: const TextStyle(
                      fontSize: 20,
                      color: lightGrayText,
                    ),
                  ),
                  const SizedBox(height: 40),
                  const Text(
                    '카드 선택',
                    style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold, color: darkGrayText),
                  ),
                  const SizedBox(height: 16),
                  _buildCardList(),
                  const SizedBox(height: 40),
                  const Text(
                    '간편 결제',
                    style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold, color: darkGrayText),
                  ),
                  const SizedBox(height: 16),
                  _buildPayButtons(),
                ],
              ),
            ),
      bottomNavigationBar: Padding(
        padding: const EdgeInsets.all(20.0),
        child: ElevatedButton(
          onPressed: isProcessing || _selectedPaymentMethod != '신용카드' ? null : simulatePayment,
          style: ElevatedButton.styleFrom(
            backgroundColor: tossBlue,
            padding: const EdgeInsets.symmetric(vertical: 16),
            shape: RoundedRectangleBorder(
              borderRadius: BorderRadius.circular(12),
            ),
            elevation: 0,
            disabledBackgroundColor: Colors.grey[300],
          ),
          child: const Text(
            '결제하기',
            style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold, color: Colors.white),
          ),
        ),
      ),
    );
  }
}