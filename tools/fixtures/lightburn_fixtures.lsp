;; SPDX-License-Identifier: BSD-3-Clause
;; Copyright (c) 2026, Sadie Forbes
;;
;; The LightBurn round-trip fixtures of docs/lightburn-fixtures.md, drawn in
;; NotoCAD. Each (Fnn "out.dxf") call builds one drawing from scratch in the
;; current session and exports it, so run ONE fixture per ncad invocation:
;;
;;   ncad -q tools/fixtures/lightburn_fixtures.lsp -e '(F01 "F01-baseline.dxf")'
;;
;; tools/fixtures/make_lightburn_fixtures.sh runs them all.
;;
;; Every entity is drawn on a layer named PEN<n> whose colour is ACI <n>, and is
;; itself BYLAYER, so LightBurn's colour-to-layer import and ours see the same
;; colour. Millimetres throughout. R12 except F08, whose SPLINE only survives in
;; R2000 (R12 has no SPLINE entity and NotoCAD degrades it to a polyline).

(defun pen (n)
  ;; Make (and set current) layer PEN<n> in colour n.
  (command "LAYER" "M" (strcat "PEN" (itoa n)) "C" (itoa n) (strcat "PEN" (itoa n)) ""))

(defun sq (x y s)
  (command "PLINE" (list x y) (list (+ x s) y) (list (+ x s) (+ y s)) (list x (+ y s)) "C"))

(defun ln (x1 y1 x2 y2)
  (command "LINE" (list x1 y1) (list x2 y2) ""))

(defun pl (pts)
  ;; An open polyline through a list of points.
  (apply 'command (append (list "PLINE") pts (list ""))))

(defun out (path ver)
  (command "DXFOUT" path ver)
  (princ (strcat "wrote " path "\n")))

;; The 40 mm square every settings-only fixture shares.
(defun square40 (path)
  (pen 1)
  (sq 0.0 0.0 40.0)
  (out path "R12"))

(defun F01 (p) (square40 p))  ; baseline
(defun F02 (p) (square40 p))  ; passes
(defun F03 (p) (square40 p))  ; start from absolute
(defun F04 (p) (square40 p))  ; start from user origin
(defun F05 (p) (square40 p))  ; job origin at centre
(defun F06 (p) (square40 p))  ; job origin at other corners and an edge
(defun F13 (p) (square40 p))  ; min/max power, speed
(defun F14 (p) (square40 p))  ; tube 2

;; Circles of 2, 10, 50 and 200 mm diameter in a row, 10 mm apart.
(defun F07 (p)
  (pen 1)
  (command "CIRCLE" (list 1.0 100.0) 1.0)
  (command "CIRCLE" (list 17.0 100.0) 5.0)
  (command "CIRCLE" (list 57.0 100.0) 25.0)
  (command "CIRCLE" (list 192.0 100.0) 100.0)
  (out p "R12"))

;; An S-curve SPLINE about 60 mm long, and a three-point ARC beside it.
(defun F08 (p)
  (pen 1)
  (entmake (list (cons 0 "SPLINE") (cons 8 "PEN1") (cons 70 8) (cons 71 3)
                 (cons 72 8) (cons 73 4)
                 (cons 40 0.0) (cons 40 0.0) (cons 40 0.0) (cons 40 0.0)
                 (cons 40 1.0) (cons 40 1.0) (cons 40 1.0) (cons 40 1.0)
                 (list 10 0.0 0.0 0.0) (list 10 20.0 40.0 0.0)
                 (list 10 40.0 -20.0 0.0) (list 10 60.0 20.0 0.0)))
  (command "ARC" (list 80.0 0.0) (list 95.0 15.0) (list 110.0 0.0))
  (out p "R2000"))

;; Segment lengths around the 8.192 mm relative-move limit, and tiny steps.
(defun F09 (p / i pts x)
  (pen 1)
  (ln 0.0 0.0 100.0 0.0)          ; 100 mm: long
  (ln 0.0 10.0 0.5 10.0)          ; 0.5 mm: short
  ;; 200 steps of 0.05 mm in X along a 0.5 mm sine, so no vertex is collinear
  ;; with its neighbours and none can be simplified away.
  (setq i 0 pts nil)
  (while (<= i 200)
    (setq x (* i 0.05)
          pts (cons (list x (+ 20.0 (* 0.5 (sin x)))) pts)
          i (1+ i)))
  (pl (reverse pts))
  (ln 0.0 30.0 5.0 30.0)          ; then a 9 mm jump: beyond the relative limit
  (ln 14.0 30.0 19.0 30.0)
  (ln 0.0 40.0 5.0 40.0)          ; and an 8 mm jump: inside it
  (ln 13.0 40.0 18.0 40.0)
  (out p "R12"))

;; Nesting: a 60 mm square, a 20 mm square hole, a 5 mm circle in the hole.
(defun F10 (p)
  (pen 1)
  (sq 0.0 0.0 60.0)
  (sq 20.0 20.0 20.0)
  (command "CIRCLE" (list 30.0 30.0) 2.5)
  (out p "R12"))

;; The same 30 mm square on three pens, side by side.
(defun F11 (p)
  (pen 1) (sq 0.0 0.0 30.0)
  (pen 3) (sq 40.0 0.0 30.0)
  (pen 5) (sq 80.0 0.0 30.0)
  (out p "R12"))

;; A 10 mm square in each of a spread of ACI colours, 15 mm apart.
(defun F12 (p / x)
  (setq x 0.0)
  (foreach n '(1 2 3 4 5 6 7 8 9 10 30 50 130 250)
    (pen n)
    (sq x 0.0 10.0)
    (setq x (+ x 15.0)))
  (out p "R12"))
