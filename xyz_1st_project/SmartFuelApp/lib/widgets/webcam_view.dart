import 'package:flutter/material.dart';
import 'package:webview_flutter/webview_flutter.dart';

/// USB 웹캠 스트리밍 영상을 WebView로 보여주는 위젯
class VideoViewWidget extends StatefulWidget {
  final String serverIp;
  final String streamPort; 

  const VideoViewWidget({Key? key, required this.serverIp, required this.streamPort}) : super(key: key);

  @override
  State<VideoViewWidget> createState() => _VideoViewWidgetState();
}

class _VideoViewWidgetState extends State<VideoViewWidget> {
  late final WebViewController _controller;
  bool _hasError = false;

  /// 위젯 초기화 시 WebView 컨트롤러 설정
  @override
  void initState() {
    super.initState();

    final fullUrl = 'http://${widget.serverIp}:${widget.streamPort}/';

    _controller = WebViewController()
      ..loadRequest(Uri.parse(fullUrl));
  }

  /// 화면 UI 구성
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