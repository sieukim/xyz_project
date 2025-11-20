#!/usr/bin/env python3
"""
무료 LLM(Ollama) 기반 내레이션 서버 (FastAPI 버전)

- POST /narrate
  body: {"event": "xy_aligning", "state": {...}}
  return: {"text": "...로봇 설명 멘트..."}

Ollama:
  - localhost:11434
  - model: llama3.2:3b (이미 설치되어 있음)
"""

from fastapi import FastAPI
from pydantic import BaseModel
import requests
import json

# --------- 요청 바디 스키마 ---------
class NarrateRequest(BaseModel):
    event: str
    state: dict | None = None

# --------- FastAPI 앱 ---------
app = FastAPI()

@app.post("/narrate")
async def narrate(req: NarrateRequest):
    """
    로봇 상태(event)를 받아서
    Ollama(llama3.2:3b)에게 내레이션 문장을 생성시키는 엔드포인트
    """
    # 1) 프롬프트 구성 (간단 버전)
    prompt = f"""
너는 주유 로봇의 내레이션 AI다.
현재 로봇의 단계는 '{req.event}' 이다.
사용자에게 상황을 친절하게 한 문장으로 한국어로만 설명해라.
반말 말고 존댓말로, 너무 길지 않게.
"""

    try:
        # 2) Ollama에 스트리밍 요청
        resp = requests.post(
            "http://localhost:11434/api/generate",
            json={
                "model": "llama3.2:3b",  # 👉 여기 이름이 ollama list 에 있는 이름과 같아야 함
                "prompt": prompt,
                "stream": True
            },
            stream=True,
            timeout=30,
        )

        full_text = ""

        # 3) 스트리밍으로 오는 JSON 라인들을 하나씩 이어붙이기
        for line in resp.iter_lines():
            if not line:
                continue
            try:
                obj = json.loads(line.decode("utf-8"))
            except Exception:
                continue

            chunk = obj.get("response", "")
            full_text += chunk

            # done 플래그 있으면 종료
            if obj.get("done"):
                break

        full_text = full_text.strip()
        print("[LLM RAW TEXT]", full_text)

        # 4) 비어 있으면 fallback 멘트
        if not full_text:
            return {"text": "안내 멘트를 가져오지 못했습니다."}

        # 5) 최종 텍스트 반환
        return {"text": full_text}

    except Exception as e:
        print("[LLM ERROR]", e)
        return {"text": f"LLM 서버 오류: {e}"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8001)
