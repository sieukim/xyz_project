import 'package:flutter/material.dart';
import 'package:webview_flutter/webview_flutter.dart';

class VideoViewWidget extends StatefulWidget {
  // ✅ 1. 하드코딩된 IP 대신 파라미터로 받도록 변경
  final String serverIp;
  final String streamPort; // USB 웹캠 포트

  // ✅ 2. 생성자에서 serverIp를 필수로 받음
  const VideoViewWidget({Key? key, required this.serverIp, required this.streamPort}) : super(key: key);

  @override
  State<VideoViewWidget> createState() => _VideoViewWidgetState();
}

class _VideoViewWidgetState extends State<VideoViewWidget> {
  late final WebViewController _controller;
  bool _hasError = false;

  @override
  void initState() {
    super.initState();

    final fullUrl = 'http://${widget.serverIp}:${widget.streamPort}/';

    _controller = WebViewController()
      ..loadRequest(Uri.parse(fullUrl));
  }

  @override
  Widget build(BuildContext context) {
    if (_hasError) {
      return Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            const Icon(Icons.videocam_off, color: Colors.red, size: 50),
            const Text(
              'USB 카메라 연결 실패',
              textAlign: TextAlign.center,
              style: TextStyle(color: Colors.red),
            ),
            Text(
              'http://${widget.serverIp}:${widget.streamPort}/',
              style: TextStyle(color: Colors.grey.shade600, fontSize: 12),
            ),
          ],
        ),
      );
    }
    return WebViewWidget(controller: _controller);
  }
}