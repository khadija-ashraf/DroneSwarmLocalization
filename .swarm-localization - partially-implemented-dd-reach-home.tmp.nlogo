extensions [csv]

globals[
  walker-vision-angle
  number-of-exits
  right-door-width
  right-door-width-offset
  deceleration
  acceleration

  collision
  walker-per-patch

  injury-threshold
  total-injured-walkers
  dd-count-threshold


  ; localization accuracy values

  ; WiFi localization real world datafile names
  wifi-exp2-three-meter-file-name
  wifi-exp2-five-meter-file-name
  wifi-exp2-seven-meter-file-name
  wifi-exp2-ten-meter-file-name
  wifi-exp2-fifteen-meter-file-name

  avg-localize-error
  localized-dd-count

]
breed [ distress-drones dd]
breed [ helper-drones hd]

distress-drones-own [
  goal
  speed
  is-localized
  localization-error

  waiting-time
  speed-limit
  speed-min

  hit-count
  injured
]

helper-drones-own [
  speed
  localized-dd-list
]

patches-own [ popularity exit maximum-visited invisible path]

to-report exits
  report patches with [exit  = true]
end

to setup
  clear-output
  clear-all
  set number-of-exits "one-exit"
  set right-door-width 8
  set right-door-width-offset 0
  set walker-vision-angle 180

  initialize-env-setting

  ask patches [
    set path false
  ]
  draw-path
  set dd-count-threshold count patches with [ path = true ]

  if dd-count > dd-count-threshold
  [
    set dd-count  dd-count-threshold
  ]
  ;;;;;;;;;

  create-DD
  ; Introduce a HD in the space
  create-hidden-HD

  reset-ticks
  reset-timer  ;keep track of seconds in netlogo
end

to go
  ; DDs hovering in the search space
   hover
  ; TODO: start the scanning of HD to find DDs
;  scan-dd
  tick

;  ;;;;;;;;;;; Stopping condition
  if localized-dd-count = dd-count[
    show avg-localize-error / (count distress-drones)
    write "Total Elapsed Time :"
    print timer
    stop  ; stop if there is no turtle left
  ]

  if count distress-drones = 0 [
    write "Total Elapsed Time :"
    print timer
    stop  ; stop if there is no turtle left
  ]

end

to scan-dd
  ask helper-drones [
    let the-current-HD self  ; this is the current HD in the agentset 'helper-drones'
    ; Scanning-area: neighboring patches inside the area of vision-angle and vision-distance
    let neighboring-patchset patches in-cone walker-vision-dist walker-vision-angle with [path != false]

    ; any distress drone found inside the scanning area
    let neighboring-dd (distress-drones-on neighboring-patchset)

    ; any distress drone that is found inside the scanning area and not yet localized
    let neighboring-unlocalized-dd (neighboring-dd with [is-localized = false])

    ;;;;;;;;;;;;; https://stackoverflow.com/questions/22121703/nested-ask-referencing-different-breeds-in-netlogo
    ; 'myself' refers to the agent that is calling the current agent (it's probably the most confusingly named primitive in NetLogo).
    ; 'self' refers to the current agent. It looks like you're using 'myself' where you should be using 'self'.
    ; However, [ some-variable ] of 'self' is the same as some-variable, so you rarely use 'self'.
    ; NetLogo automatically figures out who's variable you're referring to from the context.

    ifelse any? neighboring-unlocalized-dd[ ; found one or more unlocalized dd
      let distances-btwn-HD-to-neigh-DDs [distance myself] of neighboring-unlocalized-dd

      ask neighboring-unlocalized-dd[
        let the-current-DD self  ; this is the current DD in the agentset 'neighboring-unlocalized-dd'
        set color green
        set is-localized true
        ;;;;;;;;TODO: once we localize a DD we setup a localization accuracy value to the DD
        ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        ; 'patch-distance-btwn-HD-DD' is the distance between the HD and DD when the DD is discovered and localized by the HD
        ; the distance value can go upto the value of VISION_DISTANCE, not more than that. Because
        ; in each scan a HD can only see/discover any DDs those are located within its VISION_DISTANCE range.
        ; i.e., if VISION_DISTANCE = 3 then the distance will remain between ~0.0 to ~3.5 (approx).
        ; We round up when decimal value is > 0.5 and floor down when decimal value is < 0.5
        ;        show myself
        ;        show self

        let patch-distance-btwn-HD-DD [distance the-current-DD] of the-current-HD
        show "patch-distance-btwn-HD-DD"
        show patch-distance-btwn-HD-DD
        ifelse ((patch-distance-btwn-HD-DD - (floor patch-distance-btwn-HD-DD)) >= 0.5 ) [
          ; do a ceiling
          set patch-distance-btwn-HD-DD ceiling patch-distance-btwn-HD-DD
        ] [
          ; do a floor
          set patch-distance-btwn-HD-DD floor patch-distance-btwn-HD-DD
          if (patch-distance-btwn-HD-DD = 0) [
            set patch-distance-btwn-HD-DD 1
          ]
        ]
;        set patch-distance-btwn-HD-DD floor patch-distance-btwn-HD-DD
        show "patch-distance-btwn-HD-DD after truncate"
        show patch-distance-btwn-HD-DD
        ; call a function to read the distance-group corresponding data file name
        let file-name read-data-file-name patch-distance-btwn-HD-DD
        show "file-name:"
        show file-name
;
        ; the list holds distance difference between ground truth (GT) GPS and estimated GPS.
        ; This distance difference is the localization error in our case
        let distance-diff-list read-GPS-accuracy file-name


        ; Now that we have a list of accuracy values for a distance group,
        ; we pick an accuracy value randomly from the, and avoid any invalid value
        set localization-error 0
        while [localization-error = "" or localization-error <= 0] [
          set localization-error one-of distance-diff-list
          show "Localization error:"
          show localization-error
        ]


        set avg-localize-error  (avg-localize-error + localization-error)
        set localized-dd-count (localized-dd-count + 1)
;        die

        ;;;;;;;;;;; Now set the localized DD move towards the destination
;        move-walkers
      ]
    ][
      let random-heading random-float 360
      rt random-heading

      let next-patch patch-ahead 1
;      face next-patch
      ; moves to the next patch only if it is a path patch
      move-to-a-path-patch next-patch
    ]
  ]
  if localized-dd-count = dd-count[
    show avg-localize-error / (count distress-drones)
    stop
  ]

;  if count distress-drones = 0 [
;    ; once all the DDs of the entire search space are localized
;    ; then we calculate our average localization error.
;    show avg-localize-error / (count distress-drones)
;
;    stop  ; stop if there is no turtle left
;  ]

end

to-report read-data-file-name [patch-distance-btwn-HD-DD]
  let file-name ""

  ; in this simulation scope, we map a single patch distance as 3 meters with our real world experiment,
  ; similarly, 2 patches = 5, 3 patches = 7, 4 patches = 10, 5 patches= 15 meters.
  (ifelse
    (patch-distance-btwn-HD-DD = 1) [ set file-name wifi-exp2-three-meter-file-name]
    (patch-distance-btwn-HD-DD = 2) [ set file-name wifi-exp2-five-meter-file-name]
    (patch-distance-btwn-HD-DD = 3) [ set file-name wifi-exp2-seven-meter-file-name]
    (patch-distance-btwn-HD-DD = 4) [ set file-name wifi-exp2-ten-meter-file-name]
    (patch-distance-btwn-HD-DD = 5) [ set file-name wifi-exp2-fifteen-meter-file-name]
  )
  report file-name
end

to initialize-env-setting
  set wifi-exp2-three-meter-file-name "final_3_m_wifi_exp2.csv"
  set wifi-exp2-five-meter-file-name "final_5_m_wifi_exp2.csv"
  set wifi-exp2-seven-meter-file-name "final_7_m_wifi_exp2.csv"
  set wifi-exp2-ten-meter-file-name "final_10_m_wifi_exp2.csv"
  set wifi-exp2-fifteen-meter-file-name "final_15_m_wifi_exp2.csv"

  ; setting vision_angle based on localization mode
  ifelse (localization-mode = "Camera Localization") [
    set walker-vision-angle 180
  ]
  [
    set walker-vision-angle 360
  ]
end


to-report read-GPS-accuracy [file-name]
  let data-file csv:from-file file-name

  ; Starting from index 1 in the file to avoid the header
  let i 1

  let distance-diff-list []
;  print  item 31 item 1 camera-accuracy-testset1-accuracies
;  print  item 1 camera-accuracy-testset1-accuracies
;  print  sublist camera-accuracy-testset1-accuracies 1 3

    while [i < length data-file] [
      let column-item item 31 item i data-file
      set i (i + 1)

      set distance-diff-list lput column-item distance-diff-list
  ]
  report distance-diff-list
end


to introduce-HD
  ; introduce the HDs upon a button click
  ask helper-drones [show-turtle]
end

to hover
  ask distress-drones
  [
    ; head in a random direction
    let random-heading random-float 360
    rt random-heading
    ; moves to the next patch only if it is a path patch
    let next-patch patch-ahead 1
    move-to-a-path-patch next-patch

  ]
end

to move-to-a-path-patch [next-patch]
  ifelse next-patch != nobody and ([path] of next-patch) = true and not any? turtles-on next-patch [
      repeat 100 [
        fd speed / 100
      ]
  ] [rt heading + 180]
end


to create-hidden-HD
  ask one-of patches with [path = true] [
    sprout-helper-drones 1 [
      set hidden? true
      set color red
      set speed 1
    ]
  ]
end

to create-DD
  ask n-of dd-count patches with [path = true] [
    sprout-distress-drones 1 [
      set color blue
      set speed 1
      set is-localized false
      set goal min-one-of exits [distance myself]
    ]
  ]
end


to draw-path
  ; this block draws the outer white boundary of the ractangle of patches
  ask patches with [pxcor >= -13 and pxcor <= 13 and pycor >= -13 and pycor <= 13]
  [
    set path true
	  set pcolor white
  ]

  if number-of-exits = "one-exit" [
    ; this block draws the right exit boundary of the ractangle of patches
    ask patches with [pxcor >= 13 and pxcor <= 16 and pycor >= (- right-door-width / 2) + right-door-width-offset and pycor <= ( right-door-width / 2) + right-door-width-offset] ; Change here
    [
      set path true
      set pcolor white
    ]

    ; this block sets the exit flag true to the right exit patches
    ask patches with [pxcor = 16 and pycor >=  (- right-door-width / 2) + right-door-width-offset and pycor <= ( right-door-width / 2) + right-door-width-offset]
    [
      set exit true
      set pcolor red
    ]
  ]
end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to move-walkers
  ask distress-drones [
    ifelse patch-here = goal [
      face goal
        repeat 100 [
          fd speed / 100
        ]
      die
    ] [
      walk-towards-goal goal
      set goal min-one-of exits [distance myself] ; keep updating the goal on each step
    ]
  ]
end

to walk-towards-goal [destination]
  if maximum-visited != true [
    ; boost the popularity of the patch we're on
    ask patch-here [ become-more-popular ]
  ]

  let next-patch best-way-to destination
  ;;;;;;;;;;;;;;;;;;;;;

  let walker-ahead one-of turtles-on next-patch

  let min-patch nobody
  ifelse walker-ahead != nobody [
    slow-down-car walker-ahead
    ; try to turn to pass the blocker walker
    let empty-neighbor-patches neighbors with [ path = true and  not any? distress-drones-here ]
    ;let empty-neighbor-patches patches in-cone walker-vision-dist walker-vision-angle with [ path = true and  not any? walkers-here ]

    set min-patch one-of empty-neighbor-patches
    ;set  min-patch min-one-of empty-neighbor-patches [distance destination]

  ]
  [ ;; otherwise, speed up
    speed-up-car
  ]
  if (min-patch != nobody) [
    set next-patch min-patch

    if speed = 0 [
      set waiting-time waiting-time + 1
    ]

    if (waiting-time > 4) [
      set speed speed-limit
      set waiting-time 0
    ]
  ]
  ;;;;;;;;;;;;;;;;;;;;;;

  face next-patch
  repeat 100 [
    fd speed / 100
  ]

  ;keep track of collision
  set walker-per-patch count distress-drones-here
  if (walker-per-patch > 1) [
    set collision collision + 1;
    ;show walker-per-patch

    ask distress-drones-here [
      set hit-count hit-count + 1

      if hit-count > injury-threshold [
        set total-injured-walkers total-injured-walkers + 1
        set hit-count -99

      ]
    ]
  ]
end

to decay-popularity
  ask patches with [ not any? distress-drones-here] [
    set popularity popularity * (100 - popularity-decay-rate) / 100
    ; when popularity is below 1, the patch becomes (or stays) grass
    if popularity < 1 [ set  maximum-visited false ]
    if path = false [
      set  maximum-visited false
    ]
  ]
end

to become-more-popular
  set popularity popularity + popularity-per-step
  ; if the increase in popularity takes us above the threshold, become a route
  if popularity >= max-visiting-threshold [ set  maximum-visited true ]
end

to slow-down-car [ car-ahead ] ;; turtle procedure
  ;; slow down so you are driving more slowly than the car ahead of you
  set speed [ speed ] of car-ahead - deceleration
  if speed < speed-min [ set speed speed-min ]

end


to speed-up-car ;; turtle procedure
  set speed speed + acceleration
  if speed > speed-limit [ set speed speed-limit ]
end


to-report best-way-to [ destination ]

  ; of all the visible route patches, select the ones
  ; that would take me closer to my destination ; avoiding black patches as obstacles
  ;ask patches in-radius walker-vision-dist with [path != false] [set pcolor red  ]
  let next-patch nobody

  let visible-patches patches in-cone walker-vision-dist walker-vision-angle with [path != false]

  ; Proirity 1: select goal patches
  let visible-routes visible-patches with [exit  = true]

  ; Proirity 2: select maximum-visited patches
  ; if none of the neighbours are exit patch then chose among all most-visited patches

  if not any? visible-routes [
    set visible-routes visible-patches with [maximum-visited = true]
  ]

  ; Proirity 3: select front, left or right patches
  ; if no visited routes available then chose all white patches in the radius among the visible patches
  ; and then prioritize the front one, if no front patch available then chose either right or left one randomly


  let routes-that-take-me-closer  visible-routes with  [ invisible != true and
    distance destination <= [ distance destination - 1 ] of myself
  ]

  if any? routes-that-take-me-closer with [invisible != true] [

    set next-patch min-one-of routes-that-take-me-closer with [invisible != true] [ distance self ]

    if check-if-patch-visible next-patch = false[
      set next-patch  nobody
    ]
  ]

  ; if no neighbor patch take me closer than randomly select left or right patch as next patch
  if next-patch = nobody [
    ;show "chosing next neighbor"
    ;;;;;;;;;;;;;;;;;;;;;; start select white-patch

    let front-patch nobody
    let left-patch nobody
    let right-patch nobody
    let corner-patch1 nobody
    let corner-patch2 nobody

    if patch-ahead 1 != nobody and [path] of patch-ahead 1 = true[
      ;ask patch-ahead 1 [ set pcolor sky ]
      set front-patch patch-ahead 1
    ]
    if patch-left-and-ahead 90 1 != nobody and [path] of patch-left-and-ahead 90 1 = true [
      set left-patch patch-left-and-ahead 90 1
    ]

    if patch-right-and-ahead 90 1 != nobody and [path] of patch-right-and-ahead 90 1 = true [
      set right-patch patch-right-and-ahead 90 1
    ]

    if patch-at-heading-and-distance (heading + 45) 1 != nobody and [path] of patch-at-heading-and-distance (heading + 45) 1 = true [
      set corner-patch1 patch-at-heading-and-distance (heading + 45) 1
    ]

    if patch-at-heading-and-distance (heading - 45) 1 != nobody and [path] of patch-at-heading-and-distance (heading - 45) 1 = true [
      set corner-patch2 patch-at-heading-and-distance (heading - 45) 1
    ]

    if check-if-patch-visible front-patch = false [
      set front-patch  nobody
    ]
    if check-if-patch-visible left-patch = false [
      set left-patch  nobody
    ]
    if check-if-patch-visible right-patch = false [
      set right-patch  nobody
    ]

    if check-if-patch-visible corner-patch1 = false [
      set corner-patch1  nobody
    ]
    if check-if-patch-visible corner-patch2 = false [
      set corner-patch2  nobody
    ]



    let min-patch min-one-of (patch-set front-patch left-patch right-patch corner-patch1 corner-patch2) [distance destination]

    set next-patch min-patch
  ]

  if next-patch = nobody [
    set next-patch destination
  ]
  report next-patch

end

to-report check-if-patch-visible [next-patch]
  let is-visible true

  ifelse next-patch != nobody [
    let is-diagonal-patches false

    ; otherwise, from those route patches, choose the one that is the closest to me

    ; find out the delta-x and delta-y from the (x,y) coordinate of current patch (patch-here) and the next-patch
    let currX pxcor
    let currY pycor

    let nextX [pxcor] of next-patch
    let nextY [pycor] of next-patch

    let deltaX  nextX - currX
    let deltaY  nextY - currY

    if deltaX != 0 and deltaY != 0 [
      set is-diagonal-patches true
    ]

    if is-diagonal-patches [

      let is-right false
      let is-left false
      let is-down false
      let is-up false

      ifelse deltaX > 0 [  ; next-patch is on right
                           ; keep adding 1 go the currX get the nextX

        set is-right true

      ] [ ; next-patch is on left
          ; keep subtracting 1 go the currX get the nextX

        set is-left true
      ]

      ifelse deltaY > 0 [ ; next-patch is on top
                          ; keep subtracting 1 go the currY get the nextY

        set is-down true
      ] [ ; next-patch is on bottom
          ; keep adding 1 go the currY get the nextY

        set is-up true
      ]

      let adjacentX currX
      let adjacentY currY

      if (is-right and is-down) [
        set adjacentX currX + 1
        set adjacentY nextY - 1
      ]
      if (is-left and is-down) [
        set adjacentX currX - 1
        set adjacentY nextY - 1
      ]
      if (is-right and is-up) [
        set adjacentX currX + 1
        set adjacentY nextY + 1

      ]
      if (is-left and is-up) [
        set adjacentX currX - 1
        set adjacentY nextY + 1
      ]

      if ([path] of patch adjacentX currY = false or [path] of patch nextX adjacentY = false)  [
        set is-visible false
      ]
    ]
  ] [
    set is-visible false
  ]
  report is-visible
end
@#$#@#$#@
GRAPHICS-WINDOW
210
10
647
448
-1
-1
13.0
1
10
1
1
1
0
0
0
1
-16
16
-16
16
1
1
1
ticks
30.0

BUTTON
31
39
94
72
NIL
setup\n
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
142
44
205
77
NIL
go
T
1
T
OBSERVER
NIL
NIL
NIL
NIL
0

SLIDER
14
107
186
140
dd-count
dd-count
5
100
6.0
1
1
NIL
HORIZONTAL

BUTTON
47
156
151
189
NIL
introduce-HD
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

SLIDER
13
337
185
370
walker-vision-dist
walker-vision-dist
0
5
1.0
1
1
NIL
HORIZONTAL

BUTTON
58
215
138
248
NIL
scan-dd\n
T
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

SLIDER
11
391
183
424
popularity-decay-rate
popularity-decay-rate
0
100
8.0
1
1
NIL
HORIZONTAL

SLIDER
10
436
182
469
popularity-per-step
popularity-per-step
0
100
21.0
1
1
NIL
HORIZONTAL

SLIDER
11
479
183
512
max-visiting-threshold
max-visiting-threshold
0
100
78.0
1
1
NIL
HORIZONTAL

CHOOSER
17
277
175
322
localization-mode
localization-mode
"Camera Localization" "WiFi Localization"
0

@#$#@#$#@
## WHAT IS IT?

(a general understanding of what the model is trying to show or explain)

## HOW IT WORKS

(what rules the agents use to create the overall behavior of the model)

## HOW TO USE IT

(how to use the model, including a description of each of the items in the Interface tab)

## THINGS TO NOTICE

(suggested things for the user to notice while running the model)

## THINGS TO TRY

(suggested things for the user to try to do (move sliders, switches, etc.) with the model)

## EXTENDING THE MODEL

(suggested things to add or change in the Code tab to make the model more complicated, detailed, accurate, etc.)

## NETLOGO FEATURES

(interesting or unusual features of NetLogo that the model uses, particularly in the Code tab; or where workarounds were needed for missing features)

## RELATED MODELS

(models in the NetLogo Models Library and elsewhere which are of related interest)

## CREDITS AND REFERENCES

(a reference to the model's URL on the web if it has one, as well as any other necessary credits, citations, and links)
@#$#@#$#@
default
true
0
Polygon -7500403 true true 150 5 40 250 150 205 260 250

airplane
true
0
Polygon -7500403 true true 150 0 135 15 120 60 120 105 15 165 15 195 120 180 135 240 105 270 120 285 150 270 180 285 210 270 165 240 180 180 285 195 285 165 180 105 180 60 165 15

arrow
true
0
Polygon -7500403 true true 150 0 0 150 105 150 105 293 195 293 195 150 300 150

box
false
0
Polygon -7500403 true true 150 285 285 225 285 75 150 135
Polygon -7500403 true true 150 135 15 75 150 15 285 75
Polygon -7500403 true true 15 75 15 225 150 285 150 135
Line -16777216 false 150 285 150 135
Line -16777216 false 150 135 15 75
Line -16777216 false 150 135 285 75

bug
true
0
Circle -7500403 true true 96 182 108
Circle -7500403 true true 110 127 80
Circle -7500403 true true 110 75 80
Line -7500403 true 150 100 80 30
Line -7500403 true 150 100 220 30

butterfly
true
0
Polygon -7500403 true true 150 165 209 199 225 225 225 255 195 270 165 255 150 240
Polygon -7500403 true true 150 165 89 198 75 225 75 255 105 270 135 255 150 240
Polygon -7500403 true true 139 148 100 105 55 90 25 90 10 105 10 135 25 180 40 195 85 194 139 163
Polygon -7500403 true true 162 150 200 105 245 90 275 90 290 105 290 135 275 180 260 195 215 195 162 165
Polygon -16777216 true false 150 255 135 225 120 150 135 120 150 105 165 120 180 150 165 225
Circle -16777216 true false 135 90 30
Line -16777216 false 150 105 195 60
Line -16777216 false 150 105 105 60

car
false
0
Polygon -7500403 true true 300 180 279 164 261 144 240 135 226 132 213 106 203 84 185 63 159 50 135 50 75 60 0 150 0 165 0 225 300 225 300 180
Circle -16777216 true false 180 180 90
Circle -16777216 true false 30 180 90
Polygon -16777216 true false 162 80 132 78 134 135 209 135 194 105 189 96 180 89
Circle -7500403 true true 47 195 58
Circle -7500403 true true 195 195 58

circle
false
0
Circle -7500403 true true 0 0 300

circle 2
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240

cow
false
0
Polygon -7500403 true true 200 193 197 249 179 249 177 196 166 187 140 189 93 191 78 179 72 211 49 209 48 181 37 149 25 120 25 89 45 72 103 84 179 75 198 76 252 64 272 81 293 103 285 121 255 121 242 118 224 167
Polygon -7500403 true true 73 210 86 251 62 249 48 208
Polygon -7500403 true true 25 114 16 195 9 204 23 213 25 200 39 123

cylinder
false
0
Circle -7500403 true true 0 0 300

dot
false
0
Circle -7500403 true true 90 90 120

face happy
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 255 90 239 62 213 47 191 67 179 90 203 109 218 150 225 192 218 210 203 227 181 251 194 236 217 212 240

face neutral
false
0
Circle -7500403 true true 8 7 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Rectangle -16777216 true false 60 195 240 225

face sad
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 168 90 184 62 210 47 232 67 244 90 220 109 205 150 198 192 205 210 220 227 242 251 229 236 206 212 183

fish
false
0
Polygon -1 true false 44 131 21 87 15 86 0 120 15 150 0 180 13 214 20 212 45 166
Polygon -1 true false 135 195 119 235 95 218 76 210 46 204 60 165
Polygon -1 true false 75 45 83 77 71 103 86 114 166 78 135 60
Polygon -7500403 true true 30 136 151 77 226 81 280 119 292 146 292 160 287 170 270 195 195 210 151 212 30 166
Circle -16777216 true false 215 106 30

flag
false
0
Rectangle -7500403 true true 60 15 75 300
Polygon -7500403 true true 90 150 270 90 90 30
Line -7500403 true 75 135 90 135
Line -7500403 true 75 45 90 45

flower
false
0
Polygon -10899396 true false 135 120 165 165 180 210 180 240 150 300 165 300 195 240 195 195 165 135
Circle -7500403 true true 85 132 38
Circle -7500403 true true 130 147 38
Circle -7500403 true true 192 85 38
Circle -7500403 true true 85 40 38
Circle -7500403 true true 177 40 38
Circle -7500403 true true 177 132 38
Circle -7500403 true true 70 85 38
Circle -7500403 true true 130 25 38
Circle -7500403 true true 96 51 108
Circle -16777216 true false 113 68 74
Polygon -10899396 true false 189 233 219 188 249 173 279 188 234 218
Polygon -10899396 true false 180 255 150 210 105 210 75 240 135 240

house
false
0
Rectangle -7500403 true true 45 120 255 285
Rectangle -16777216 true false 120 210 180 285
Polygon -7500403 true true 15 120 150 15 285 120
Line -16777216 false 30 120 270 120

leaf
false
0
Polygon -7500403 true true 150 210 135 195 120 210 60 210 30 195 60 180 60 165 15 135 30 120 15 105 40 104 45 90 60 90 90 105 105 120 120 120 105 60 120 60 135 30 150 15 165 30 180 60 195 60 180 120 195 120 210 105 240 90 255 90 263 104 285 105 270 120 285 135 240 165 240 180 270 195 240 210 180 210 165 195
Polygon -7500403 true true 135 195 135 240 120 255 105 255 105 285 135 285 165 240 165 195

line
true
0
Line -7500403 true 150 0 150 300

line half
true
0
Line -7500403 true 150 0 150 150

pentagon
false
0
Polygon -7500403 true true 150 15 15 120 60 285 240 285 285 120

person
false
0
Circle -7500403 true true 110 5 80
Polygon -7500403 true true 105 90 120 195 90 285 105 300 135 300 150 225 165 300 195 300 210 285 180 195 195 90
Rectangle -7500403 true true 127 79 172 94
Polygon -7500403 true true 195 90 240 150 225 180 165 105
Polygon -7500403 true true 105 90 60 150 75 180 135 105

plant
false
0
Rectangle -7500403 true true 135 90 165 300
Polygon -7500403 true true 135 255 90 210 45 195 75 255 135 285
Polygon -7500403 true true 165 255 210 210 255 195 225 255 165 285
Polygon -7500403 true true 135 180 90 135 45 120 75 180 135 210
Polygon -7500403 true true 165 180 165 210 225 180 255 120 210 135
Polygon -7500403 true true 135 105 90 60 45 45 75 105 135 135
Polygon -7500403 true true 165 105 165 135 225 105 255 45 210 60
Polygon -7500403 true true 135 90 120 45 150 15 180 45 165 90

sheep
false
15
Circle -1 true true 203 65 88
Circle -1 true true 70 65 162
Circle -1 true true 150 105 120
Polygon -7500403 true false 218 120 240 165 255 165 278 120
Circle -7500403 true false 214 72 67
Rectangle -1 true true 164 223 179 298
Polygon -1 true true 45 285 30 285 30 240 15 195 45 210
Circle -1 true true 3 83 150
Rectangle -1 true true 65 221 80 296
Polygon -1 true true 195 285 210 285 210 240 240 210 195 210
Polygon -7500403 true false 276 85 285 105 302 99 294 83
Polygon -7500403 true false 219 85 210 105 193 99 201 83

square
false
0
Rectangle -7500403 true true 30 30 270 270

square 2
false
0
Rectangle -7500403 true true 30 30 270 270
Rectangle -16777216 true false 60 60 240 240

star
false
0
Polygon -7500403 true true 151 1 185 108 298 108 207 175 242 282 151 216 59 282 94 175 3 108 116 108

target
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240
Circle -7500403 true true 60 60 180
Circle -16777216 true false 90 90 120
Circle -7500403 true true 120 120 60

tree
false
0
Circle -7500403 true true 118 3 94
Rectangle -6459832 true false 120 195 180 300
Circle -7500403 true true 65 21 108
Circle -7500403 true true 116 41 127
Circle -7500403 true true 45 90 120
Circle -7500403 true true 104 74 152

triangle
false
0
Polygon -7500403 true true 150 30 15 255 285 255

triangle 2
false
0
Polygon -7500403 true true 150 30 15 255 285 255
Polygon -16777216 true false 151 99 225 223 75 224

truck
false
0
Rectangle -7500403 true true 4 45 195 187
Polygon -7500403 true true 296 193 296 150 259 134 244 104 208 104 207 194
Rectangle -1 true false 195 60 195 105
Polygon -16777216 true false 238 112 252 141 219 141 218 112
Circle -16777216 true false 234 174 42
Rectangle -7500403 true true 181 185 214 194
Circle -16777216 true false 144 174 42
Circle -16777216 true false 24 174 42
Circle -7500403 false true 24 174 42
Circle -7500403 false true 144 174 42
Circle -7500403 false true 234 174 42

turtle
true
0
Polygon -10899396 true false 215 204 240 233 246 254 228 266 215 252 193 210
Polygon -10899396 true false 195 90 225 75 245 75 260 89 269 108 261 124 240 105 225 105 210 105
Polygon -10899396 true false 105 90 75 75 55 75 40 89 31 108 39 124 60 105 75 105 90 105
Polygon -10899396 true false 132 85 134 64 107 51 108 17 150 2 192 18 192 52 169 65 172 87
Polygon -10899396 true false 85 204 60 233 54 254 72 266 85 252 107 210
Polygon -7500403 true true 119 75 179 75 209 101 224 135 220 225 175 261 128 261 81 224 74 135 88 99

wheel
false
0
Circle -7500403 true true 3 3 294
Circle -16777216 true false 30 30 240
Line -7500403 true 150 285 150 15
Line -7500403 true 15 150 285 150
Circle -7500403 true true 120 120 60
Line -7500403 true 216 40 79 269
Line -7500403 true 40 84 269 221
Line -7500403 true 40 216 269 79
Line -7500403 true 84 40 221 269

wolf
false
0
Polygon -16777216 true false 253 133 245 131 245 133
Polygon -7500403 true true 2 194 13 197 30 191 38 193 38 205 20 226 20 257 27 265 38 266 40 260 31 253 31 230 60 206 68 198 75 209 66 228 65 243 82 261 84 268 100 267 103 261 77 239 79 231 100 207 98 196 119 201 143 202 160 195 166 210 172 213 173 238 167 251 160 248 154 265 169 264 178 247 186 240 198 260 200 271 217 271 219 262 207 258 195 230 192 198 210 184 227 164 242 144 259 145 284 151 277 141 293 140 299 134 297 127 273 119 270 105
Polygon -7500403 true true -1 195 14 180 36 166 40 153 53 140 82 131 134 133 159 126 188 115 227 108 236 102 238 98 268 86 269 92 281 87 269 103 269 113

x
false
0
Polygon -7500403 true true 270 75 225 30 30 225 75 270
Polygon -7500403 true true 30 75 75 30 270 225 225 270
@#$#@#$#@
NetLogo 6.2.2
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
default
0.0
-0.2 0 0.0 1.0
0.0 1 1.0 0.0
0.2 0 0.0 1.0
link direction
true
0
Line -7500403 true 150 150 90 180
Line -7500403 true 150 150 210 180
@#$#@#$#@
0
@#$#@#$#@
