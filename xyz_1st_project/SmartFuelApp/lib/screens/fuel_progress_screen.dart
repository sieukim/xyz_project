import 'dart:async';
import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:flutter_tts/flutter_tts.dart';
import 'package:http/http.dart' as http;

import 'package:smart_fuel/widgets/realsense_view.dart';
import 'package:smart_fuel/widgets/webcam_view.dart';

class FuelProgressScreen extends StatefulWidget {
  final String orderId;
  final String rosBaseUrl;

  const FuelProgressScreen(
      {Key? key, required this.orderId, required this.rosBaseUrl})
      : super(key: key);

  @override
  State<FuelProgressScreen> createState() => _FuelProgressScreenState();
}

class _FuelProgressScreenState extends State<FuelProgressScreen> {
  final FlutterTts _flutterTts = FlutterTts();
  Timer? _timer;
  String _status = '대기 중';
  int _progress = 0;
  bool _completed = false;
  int _countdown = 5;
  Timer? _countdownTimer;

  String? _serverIp;

  @override
  void initState() {
    super.initState();
    _initTtsAndSpeak();
    _extractIpAndStartPolling();
  }

  Future<void> _initTtsAndSpeak() async {
    await _flutterTts.setLanguage('ko-KR');
    await _flutterTts.setSpeechRate(1.0);
    await _flutterTts.speak("주유중입니다.");
  }

  void _extractIpAndStartPolling() {
    try {
      final uri = Uri.parse(widget.rosBaseUrl);
      setState(() {
        _serverIp = uri.host;
      });

      _fetchStatus();
      _timer = Timer.periodic(const Duration(seconds: 2), (_) => _fetchStatus());
    } catch (e) {
      debugPrint("Invalid rosBaseUrl: $e");
      setState(() {
        _status = "서버 URL 오류";
      });
    }
  }

  @override
  void dispose() {
    _timer?.cancel();
    _countdownTimer?.cancel();
    _flutterTts.stop();
    super.dispose();
  }

  Future<void> _fetchStatus() async {
    if (_serverIp == null) {
      debugPrint("Server IP not yet extracted, skipping poll.");
      return;
    }

    final statusUrl = Uri.parse('http://$_serverIp:8000/status/${widget.orderId}');

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
      }
    } catch (e) {
      // 네트워크 에러는 무시
    }
  }

  Future<void> _onCompleted() async {
    if (_completed) return;
    _timer?.cancel();

    await _flutterTts.speak("주유를 완료했습니다. 안녕히 가세요.");

    if (!mounted) return;
    setState(() {
      _completed = true;
      _status = '주유 완료';
      _progress = 100;
    });

    _countdownTimer = Timer.periodic(const Duration(seconds: 1), (timer) {
      if (_countdown > 1) {
        setState(() {
          _countdown--;
        });
      } else {
        timer.cancel();
        _navigateToHome();
      }
    });
  }

  void _navigateToHome() {
    if (mounted) {
      _countdownTimer?.cancel();
      Navigator.popUntil(context, (route) => route.isFirst);
    }
  }

  @override
  Widget build(BuildContext context) {
    const tossBlue = Color(0xFF3182F7);
    const darkGrayText = Color(0xFF333D4B);
    const lightGrayText = Color(0xFF6B7684);
    const lightGrayBg = Color(0xFFF2F4F6);
    const white = Colors.white;

    return Scaffold(
      backgroundColor: white,
      appBar: AppBar(
        title: const Text('주유 진행 상황', style: TextStyle(color: darkGrayText)),
        backgroundColor: white,
        elevation: 0,
        iconTheme: const IconThemeData(color: darkGrayText),
        automaticallyImplyLeading: false,
      ),
      body: Padding(
        padding: const EdgeInsets.all(20),
        child: Column(
          children: [
            Expanded(
              flex: 5,
              child: _serverIp == null
                  ? const Center(child: CircularProgressIndicator(strokeWidth: 2, color: tossBlue))
                  : Column(
                      children: [
                        Expanded(
                          child: Card(
                            clipBehavior: Clip.antiAlias,
                            shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
                            elevation: 0,
                            color: lightGrayBg,
                            child: VideoViewWidget(serverIp: _serverIp!),
                          ),
                        ),
                        const SizedBox(height: 10),
                        Expanded(
                          child: Card(
                            shape: RoundedRectangleBorder(
                                borderRadius: BorderRadius.circular(12)),
                            clipBehavior: Clip.antiAlias,
                            elevation: 0,
                            color: lightGrayBg,
                            child: RealSenseViewWidget(serverIp: _serverIp!),
                          ),
                        ),
                      ],
                    ),
            ),
            const SizedBox(height: 24),
            Container(
              padding: const EdgeInsets.all(24),
              decoration: BoxDecoration(
                color: lightGrayBg,
                borderRadius: BorderRadius.circular(16),
              ),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    _completed ? '주유 완료!' : '주유 중입니다...',
                    style: const TextStyle(fontSize: 22, fontWeight: FontWeight.bold, color: darkGrayText),
                  ),
                  const SizedBox(height: 8),
                  Text(
                    '주문 ID: ${widget.orderId}',
                    style: const TextStyle(fontSize: 14, color: lightGrayText),
                  ),
                  const SizedBox(height: 20),
                  LinearProgressIndicator(
                    value: _progress / 100.0,
                    minHeight: 10,
                    backgroundColor: Colors.grey[300],
                    color: tossBlue,
                  ),
                  const SizedBox(height: 8),
                  Align(
                    alignment: Alignment.centerRight,
                    child: Text(
                      '$_progress%',
                      style: const TextStyle(fontSize: 16, fontWeight: FontWeight.bold, color: darkGrayText),
                    ),
                  ),
                ],
              ),
            ),
            const Spacer(),
          ],
        ),
      ),
      bottomNavigationBar: Padding(
        padding: const EdgeInsets.all(20.0),
        child: ElevatedButton(
          onPressed: _completed ? _navigateToHome : null,
          style: ElevatedButton.styleFrom(
            backgroundColor: tossBlue,
            foregroundColor: white,
            disabledBackgroundColor: lightGrayBg,
            disabledForegroundColor: darkGrayText.withOpacity(0.38),
            padding: const EdgeInsets.symmetric(vertical: 16),
            shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
            elevation: 0,
          ),
          child: Text(
            _completed ? '$_countdown초 후 홈으로 이동' : '주유 중입니다',
            style: const TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
          ),
        ),
      ),
    );
  }
}
