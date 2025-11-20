class AINarrator:
    """
    간단한 AI Narrator
    - event 키워드를 받아 한국어 문장으로 변환
    - 옵션으로 LLM 서버에 요청해 자연어를 생성할 수도 있음
    """
    def __init__(self, node, use_llm: bool = False, llm_url: str = "http://localhost:8001/narrate"):
        self.node = node
        self.use_llm = use_llm
        self.llm_url = llm_url

        # 이벤트별 기본 템플릿
        self.templates = {
            "start_sequence": "결제가 완료되어 {fuel_type} 주유 시퀀스를 시작합니다.",
            "orient_negative_y": "주유구를 찾기 위해 차량 방향으로 회전합니다.",
            "xy_aligning": "주유구와 로봇 손끝의 XY 정렬을 진행 중입니다.",
            "xy_aligned": "XY 정렬이 완료되었습니다. 이제 깊이 방향으로 접근을 시작합니다.",
            "depth_done": "주유구 근처까지 접근을 마쳤습니다.",
            "cap_open_start": "주유구 뚜껑을 열기 위한 회전 동작을 시작합니다.",
            "cap_open_done": "주유구 뚜껑을 연 동작을 완료했습니다.",
            "switch_to_nozzle": "{fuel_type} 주유를 위해 노즐 쪽으로 이동합니다.",
            "grab_nozzle": "주유건을 잡기 위해 그리퍼를 닫습니다.",
            "insert_nozzle": "주유건을 주유구에 꽂는 동작을 수행합니다.",
            "fueling": "설정된 횟수만큼 주유 동작을 반복 수행합니다.",
            "return_nozzle": "주유가 완료되어 주유건을 제 위치로 되돌립니다.",
            "finish": "모든 주유 시퀀스를 완료했습니다. 로봇을 홈 위치로 복귀합니다.",
            "cap_not_detected": "주유구가 인식되지 않습니다. 주유구를 더 열어주세요."
        }

    def narrate(self, event: str, **kwargs):
        """
        event: 위 templates의 키
        kwargs: fuel_type 등 포맷 변수
        """
        try:
            if self.use_llm:
                text = self._call_llm(event, **kwargs)
            else:
                text = self._format_template(event, **kwargs)

            if not text:
                return

            # MotionController 안의 say() 사용
            if hasattr(self.node, "say"):
                self.node.say(text)
            else:
                self.node.get_logger().info(f"[Narrator] {text}")

        except Exception as e:
            self.node.get_logger().warn(f"[Narrator] error: {e}")

    def _format_template(self, event: str, **kwargs) -> str:
        tmpl = self.templates.get(event)
        if not tmpl:
            return ""
        try:
            return tmpl.format(**kwargs)
        except Exception:
            return tmpl

    def _call_llm(self, event: str, **kwargs) -> str:
        import requests
        payload = {"event": event, "state": kwargs}
        resp = requests.post(self.llm_url, json=payload, timeout=15)
        data = resp.json()
        return data.get("text", "")
