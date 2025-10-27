import 'dart:async';
import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;

class FuelProgressScreen extends StatefulWidget {
  final String orderId;
  final String rosBaseUrl; // ex: http://192.168.0.100:5000

  const FuelProgressScreen(
      {Key? key, required this.orderId, required this.rosBaseUrl})
      : super(key: key);

  @override
  State<FuelProgressScreen> createState() => _FuelProgressScreenState();
}

class _FuelProgressScreenState extends State<FuelProgressScreen> {
  Timer? _timer;
  String _status = '대기 중';
  int _progress = 0; // 0..100
  bool _completed = false;

  @override
  void initState() {
    super.initState();
    // 즉시 한 번 조회하고 주기적으로 폴링
    _fetchStatus();
    _timer = Timer.periodic(const Duration(seconds: 2), (_) => _fetchStatus());
  }

  @override
  void dispose() {
    _timer?.cancel();
    super.dispose();
  }

  Future<void> _fetchStatus() async {
    final statusUrl = Uri.parse(
        '${widget.rosBaseUrl.replaceAll(RegExp(r'\/$'), '')}/status/${widget.orderId}');
    try {
      final res = await http.get(statusUrl).timeout(const Duration(seconds: 6));
      if (res.statusCode == 200) {
        final body = jsonDecode(res.body);
        if (body is Map) {
          final s = (body['status'] ?? '').toString();
          final p = int.tryParse(body['progress']?.toString() ?? '') ?? 0;
          setState(() {
            _status = s.isNotEmpty ? s : _status;
            _progress = p.clamp(0, 100);
          });

          if (s == 'completed' || s == 'done' || s == 'finished') {
            _onCompleted();
          }
        }
      } else {
        // 서버 에러는 무시하고 다음 폴링까지 대기
      }
    } catch (e) {
      // 네트워크 에러는 무시 (오프라인일 수 있음)
    }
  }

  void _onCompleted() {
    if (_completed) return;
    _completed = true;
    _timer?.cancel();
    if (!mounted) return;
    ScaffoldMessenger.of(context).showSnackBar(
      const SnackBar(content: Text('주유 완료!')),
    );
    // 1.5초 후에 루트로 이동
    Future.delayed(const Duration(milliseconds: 1500), () {
      if (!mounted) return;
      Navigator.popUntil(context, (route) => route.isFirst);
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('주유 진행 상황')),
      body: Padding(
        padding: const EdgeInsets.all(20),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Text('주문 ID: ${widget.orderId}'),
            const SizedBox(height: 12),
            Text('상태: $_status', style: const TextStyle(fontSize: 18)),
            const SizedBox(height: 12),
            SizedBox(
              width: double.infinity,
              child: LinearProgressIndicator(
                value: _progress / 100.0,
                minHeight: 12,
              ),
            ),
            const SizedBox(height: 8),
            Text('진행: $_progress%'),
            const SizedBox(height: 24),
            if (!_completed)
              ElevatedButton(
                onPressed: () {
                  // 사용자가 직접 상태 다시 조회
                  _fetchStatus();
                },
                child: const Text('상태 새로고침'),
              )
            else
              const Text('완료되었습니다.'),
          ],
        ),
      ),
    );
  }
}
