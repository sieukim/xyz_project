import 'package:flutter/material.dart';
import 'dart:convert';
import 'package:http/http.dart' as http;
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
  bool isProcessing = false;
  final String rosServerUrl = 'http://192.168.50.107:12345/start_fuel'; // 수정 필요

  Future<void> simulatePayment() async {
    setState(() => isProcessing = true);

    await Future.delayed(const Duration(seconds: 2)); // 가상 결제 지연

    try {
      // 더 구체적인 payload를 보냅니다. 서버에서 orderId를 반환하도록 기대합니다.
      final orderId =
          '${DateTime.now().millisecondsSinceEpoch}-${Random().nextInt(900000)}';
      final payload = {
        'event': 'payment_complete',
        'orderId': orderId,
        'fuelType': widget.fuelType,
        'amount': widget.amount,
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
        // 서버가 주문을 수락하고 orderId/endpoint를 반환하면 진행 화면으로 이동
        String returnedOrderId = orderId;
        try {
          final body = jsonDecode(res.body);
          if (body is Map && body['orderId'] != null) {
            returnedOrderId = body['orderId'].toString();
          }
        } catch (_) {}

        // 진행 화면으로 이동하여 로봇 상태를 폴링합니다.
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

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('가상 결제')),
      body: Center(
        child: isProcessing
            ? const CircularProgressIndicator()
            : Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  Text('주유 종류: ${widget.fuelType}'),
                  Text('결제 금액: ${widget.amount}원'),
                  const SizedBox(height: 30),
                  ElevatedButton(
                    onPressed: simulatePayment,
                    child: const Text('💳 결제 완료'),
                  ),
                ],
              ),
      ),
    );
  }
}
