# OCR source attribution

`src/ocr_engine.cpp` adapts the PPOCR detection geometry, per-line recognition
preprocessing and CTC greedy decoding from SpacemiT model-zoo-vision:

- Source: https://github.com/spacemit-com/model-zoo-vision/blob/main/src/deploy/ppocr/cpp/ppocr_detector.cpp
- Read-only snapshot SHA-256: `e0cd0fb2b5813baffdea4b6f44f653d8c96b9710ad40ecd2299334190e9e8d08` (2026-10-18).
- Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
- License: Apache-2.0, reproduced in `LICENSES/Apache-2.0.txt`.

`src/ocr_rvv_pack.hpp` retains the upstream license and implementation of
`src/operators/image_preprocess/rvv_image_pack.h` from the same repository.
Imported file SHA-256: `a9cbf2304391061784c93ce81da7f141af856df73d65f4b24b5112dd4f28beb7`.
The source URL/hash are also retained under
`riscv/test/ocr-implementation-20261018/rvv-pack-source.txt`.

Local adaptations use RGB8 directly, reusable input storage, explicit per-session
CPU0 EP settings, bounded candidates/recognition, checked model I/O and dictionary,
and cooperative stop boundaries. The project does not import VisionService or a
second OpenCV ABI. Model and dictionary files are separately supplied deployment
resources and are not included or downloaded by this package.
