# 구조도

`algorithm_flow.svg` / `algorithm_flow.png` — merge.ino의 초음파 센싱 알고리즘 구조도.
현재 코드(절대 기준거리 방식) 기준이다.

- 측정 루프 (라운드로빈, 센서당 45ms)
- 한 센서의 판정 흐름 — IDLE / HOLE / STEP 세 상태와 각 상태의 하위 판정
- 결과 처리와 시리얼 출력
- 우측: 설치 파라미터, 임계값 산출, 코드값, 판정 원리

`gen_flow.py`가 SVG를 만든다. 알고리즘이 바뀌면 이 스크립트를 고치고 다시 돌린다.

```sh
python3 docs/gen_flow.py                      # docs/algorithm_flow.svg 갱신
# PNG는 크로미움으로 렌더
chromium --headless --no-sandbox --force-device-scale-factor=2 \
  --window-size=1290,1700 --screenshot=docs/algorithm_flow.png \
  file://$PWD/docs/algorithm_flow.svg
```

PNG 렌더에는 한글 폰트가 필요하다 (`apt-get install fonts-nanum`).
SVG 자체는 시스템 폰트(맑은 고딕 등)로 대체되므로 그대로 열어도 된다.
