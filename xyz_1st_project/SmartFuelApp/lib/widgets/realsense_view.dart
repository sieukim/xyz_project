import 'package:flutter/material.dart';
import 'package:webview_flutter/webview_flutter.dart';

/// RealSense 카메라 스트리밍 영상을 WebView로 보여주는 위젯
class RealSenseViewWidget extends StatefulWidget {
  final String serverIp;
  final String streamPort; 

  const RealSenseViewWidget({Key? key, required this.serverIp, required this.streamPort}) : super(key: key);

  @override
  State<RealSenseViewWidget> createState() => _RealSenseViewWidgetState();
}

class _RealSenseViewWidgetState extends State<RealSenseViewWidget> {
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
            Text(
              'Realsense 카메라 연결 실패',
              textAlign: TextAlign.center,
              style: const TextStyle(color: Colors.red),
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