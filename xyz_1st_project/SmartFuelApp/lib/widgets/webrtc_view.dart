import 'package:flutter/material.dart';
import 'package:flutter_webrtc/flutter_webrtc.dart';
import 'dart:convert';
import 'package:http/http.dart' as http;

class WebRTCStreamView extends StatefulWidget {
  final String serverIp; 
  final String streamName; 

  const WebRTCStreamView({
    Key? key,
    required this.serverIp,
    required this.streamName,
  }) : super(key: key);

  @override
  State<WebRTCStreamView> createState() => _WebRTCStreamViewState();
}

class _WebRTCStreamViewState extends State<WebRTCStreamView> {
  final _renderer = RTCVideoRenderer();
  RTCPeerConnection? _pc;
  String _status = "초기화 중...";

  @override
  void initState() {
    super.initState();
    _initRenderer();
  }

  Future<void> _initRenderer() async {
    await _renderer.initialize();
    _connect();
  }

  @override
  void dispose() {
    _renderer.dispose();
    _pc?.close();
    super.dispose();
  }

  @override
  void didUpdateWidget(covariant WebRTCStreamView oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.serverIp != widget.serverIp ||
        oldWidget.streamName != widget.streamName) {
      _pc?.close();
      _renderer.srcObject = null;
      _connect();
    }
  }

  Future<void> _connect() async {
    final whepUrl = 'http://${widget.serverIp}:8889/${widget.streamName}/whep';
    setState(() => _status = "연결 시도... (${widget.streamName})");

    try {
      _pc = await createPeerConnection({'iceServers': []});

      _pc!.addTransceiver(
        kind: RTCRtpMediaType.RTCRtpMediaTypeVideo,
        init: RTCRtpTransceiverInit(direction: TransceiverDirection.RecvOnly),
      );

      _pc!.onTrack = (event) {
        if (event.track.kind == 'video') {
          setState(() {
            _renderer.srcObject = event.streams[0];
            _status = "스트리밍 중";
          });
        }
      };

      final offer = await _pc!.createOffer();
      await _pc!.setLocalDescription(offer);

      final response = await http.post(
        Uri.parse(whepUrl),
        headers: {'Content-Type': 'application/sdp'},
        body: offer.sdp,
      );

      if (response.statusCode != 201) {
        throw Exception('mediamtx 서버 연결 실패: ${response.statusCode}');
      }

      await _pc!.setRemoteDescription(
        RTCSessionDescription(response.body, 'answer'),
      );
    } catch (e) {
      debugPrint("❌ ${widget.streamName} 연결 실패: $e");
      setState(() => _status = "연결 실패");
    }
  }

  @override
  Widget build(BuildContext context) {
    return Container(
      color: Colors.black,
      child: Stack(
        alignment: Alignment.bottomLeft,
        children: [
          RTCVideoView(
            _renderer,
            objectFit: RTCVideoViewObjectFit.RTCVideoViewObjectFitCover,
            mirror: (widget.streamName == 'webcam'),
          ),
          Positioned(
            bottom: 8,
            left: 8,
            child: Container(
              padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 2),
              color: Colors.black54,
              child: Text(
                _status,
                style: const TextStyle(color: Colors.white, fontSize: 12),
              ),
            ),
          )
        ],
      ),
    );
  }
}