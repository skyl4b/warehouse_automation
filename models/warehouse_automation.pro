<Project application="DESTool" version="0.83">

<VariablePool>
<Variable>
G_TaskTransmitter  System        +Visual+     
<Value>
<VioSystem>
<Generator name="G_TaskTransmitter" ftype="System">

<Alphabet>
<Event name="tin_k">
<Controllable/>
</Event>
<Event name="broadcast"/>
<Event name="request_i">
<Controllable/>
</Event>
<Event name="accept_i">
<Controllable/>
</Event>
<Event name="tout_k">
<Controllable/>
</Event>
<Event name="deny_i">
<Controllable/>
</Event>
</Alphabet>

<StateSet>
<State id="1" name="Idle">
<Initial/>
<Marked/>
</State>
<State id="2" name="Broadcast"/>
<State id="3" name="Answer"/>
<State id="4" name="Reject"/>
</StateSet>

<TransitionRelation>
<Transition x1="1" event="tin_k" x2="2"/>
<Transition x1="1" event="request_i" x2="4"/>
<Transition x1="1" event="tout_k" x2="2"/>
<Transition x1="2" event="broadcast" x2="2"/>
<Transition x1="2" event="request_i" x2="3"/>
<Transition x1="3" event="accept_i" x2="1"/>
<Transition x1="3" event="deny_i" x2="2"/>
<Transition x1="4" event="deny_i" x2="1"/>
</TransitionRelation>

</Generator>
<VioModels>
<TransitionList>
=AAAACAEAAAABAAAACgB0AGkAbgBfAGsAAAACAQAAAAEAAAAMAHQAbwB1AHQAXwBrAAAAAgEAAAACAAAAEgBiAHIAbwBhAGQAYwBhAHMAdAAAAAIBAAAAAgAAABIAcgBlAHEAdQBlAHMAdABfAGkAAAADAQAAAAMAAAAQAGEAYwBjAGUAcAB0AF8AaQAAAAEBAAAAAwAAAAwAZABlAG4AeQBfAGkAAAACAQAAAAEAAAASAHIAZQBxAHUAZQBzAHQAXwBpAAAABAEAAAAEAAAADABkAGUAbgB5AF8AaQAAAAE== </TransitionList>
<StateList>
=AAAABAIAAAABAgAAAAICAAAAAwIAAAAE= </StateList>
<EventList>
=AAAABgMAAAAKAHQAaQBuAF8AawMAAAASAGIAcgBvAGEAZABjAGEAcwB0AwAAABIAcgBlAHEAdQBlAHMAdABfAGkDAAAAEABhAGMAYwBlAHAAdABfAGkDAAAADAB0AG8AdQB0AF8AawMAAAAMAGQAZQBuAHkAXwBp= </EventList>
<GraphData>
<States>
<State>
1             
<Item>
<Position>
81.393000      187.870000    
</Position>
<BasePoints>
0              0              28.615500      28.615500      -80.698560     0             
-28.650944     0             
</BasePoints>
</Item>
</State>
<State>
2             
<Item>
<Position>
277.040000     93.421000     
</Position>
<BasePoints>
0              0              53.145000      53.145000      -60            0             
-53.092632     0             
</BasePoints>
</Item>
</State>
<State>
3             
<Item>
<Position>
487.410000     140.640000    
</Position>
<BasePoints>
0              0              43.332000      43.332000      -60            0             
-43.375568     0             
</BasePoints>
</Item>
</State>
<State>
4             
<Item>
<Position>
277.040000     210.090000    
</Position>
<BasePoints>
0              0              38.426500      38.426500      -60            0             
-38.458500     0             
</BasePoints>
</Item>
</State>
</States>
<TransRel>
<Trans>
1              tin_k         2              +Free+       
<Item>
<Position>
81.393000      187.870000    
</Position>
<BasePoints>
85.557000      -93.755000     0              0              15.375814      -24.131918    
24.927000      -38.340000     37.977000      -54.310000     53.617000      -63.890000    
75.987000      -77.600000     103.657000     -85.200000     142.671363     -90.465567    
195.647000     -94.449000    
</BasePoints>
</Item>
</Trans>
<Trans>
1              tout_k        2              +Free+       
<Item>
<Position>
81.393000      187.870000    
</Position>
<BasePoints>
85.557000      -53.480000     0              0              27.260176      -8.636158     
51.307000      -17.030000     87.557000      -30.620000     117.507000     -45.840000    
124.707000     -49.500000     132.107000     -53.650000     151.085560     -65.554482    
195.647000     -94.449000    
</BasePoints>
</Item>
</Trans>
<Trans>
1              request_i     4              +Free+       
<Item>
<Position>
81.393000      187.870000    
</Position>
<BasePoints>
85.557000      -0.700000      0              0              28.476886      3.114675      
58.317000      6.550000       106.527000     12.110000      157.383496     17.929517     
195.647000     22.220000     
</BasePoints>
</Item>
</Trans>
<Trans>
2              broadcast     2              +Free+       
<Item>
<Position>
277.040000     93.421000     
</Position>
<BasePoints>
0              -85.782100     0              0              -20.270351     -49.123914    
-20.210000     -65.322000     -13.430000     -78.143000     0              -78.143000    
9.240000       -78.143000     15.320000      -72.083000     14.811626      -51.153647    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
2              request_i     3              +Free+       
<Item>
<Position>
277.040000     93.421000     
</Position>
<BasePoints>
110.090000     10.419000      0              0              52.046048      10.617394     
79.010000      16.369000      112.340000     23.639000      142.030000     30.559000     
145.970000     31.469000      150.030000     32.439000      168.278577     36.892571     
210.370000     47.219000     
</BasePoints>
</Item>
</Trans>
<Trans>
3              accept_i      1              +Free+       
<Item>
<Position>
487.410000     140.640000    
</Position>
<BasePoints>
-210.370000    128.480000     0              0              -27.265496     33.665148     
-55.400000     67.200000      -103.080000    115.800000     -157.230000    136.110000    
-240.390000    167.320000     -278.380000    142.160000     -352.400000    93.060000     
-359.740000    88.190000      -367.150000    82.370000      -384.919484    66.525104     
-406.017000    47.230000     
</BasePoints>
</Item>
</Trans>
<Trans>
3              deny_i        2              +Free+       
<Item>
<Position>
487.410000     140.640000    
</Position>
<BasePoints>
-100.280000    -6.250000      0              0              -42.921793     6.496326      
-69.030000     9.100000       -103.210000    9.810000       -132.230000    1.390000      
-140.090000    -0.890000      -147.910000    -4.240000      -167.091222    -16.451465    
-210.370000    -47.219000    
</BasePoints>
</Item>
</Trans>
<Trans>
4              deny_i        1              +Free+       
<Item>
<Position>
277.040000     210.090000    
</Position>
<BasePoints>
-110.090000    4.860000       0              0              -36.971994     10.406712     
-66.280000     17.250000      -107.540000    22.920000      -142.030000    12.500000     
-148.710000    10.480000      -155.330000    7.290000       -172.813368    -4.931211     
-195.647000    -22.220000    
</BasePoints>
</Item>
</Trans>
</TransRel>
</GraphData>
</VioModels>
<VioLayout>
0             =AAAA/wAAAAAAAAADAAADtgAAA7YAAAAAAQAAAAQBAAAAAQ===  0              150            958            1.146716       0             
958           
</VioLayout>
</VioSystem>
</Value>
</Variable>
<Variable>
G_Robot_i     System        +Visual+     
<Value>
<VioSystem>
<Generator name="G_Robot_i" ftype="System">

<Alphabet>
<Event name="work_i">
<Controllable/>
</Event>
<Event name="goal_reached_i"/>
<Event name="pick_box_i"/>
<Event name="drop_box_i"/>
</Alphabet>

<StateSet>
<State id="1" name="Idle">
<Initial/>
<Marked/>
</State>
<State id="2" name="ToStart"/>
<State id="4" name="PickBox"/>
<State id="5" name="ToEnd"/>
<State id="6" name="DropBox"/>
</StateSet>

<TransitionRelation>
<Transition x1="1" event="work_i" x2="2"/>
<Transition x1="2" event="goal_reached_i" x2="4"/>
<Transition x1="4" event="pick_box_i" x2="5"/>
<Transition x1="5" event="goal_reached_i" x2="6"/>
<Transition x1="6" event="drop_box_i" x2="1"/>
</TransitionRelation>

</Generator>
<VioModels>
<TransitionList>
=AAAABQEAAAABAAAADAB3AG8AcgBrAF8AaQAAAAIBAAAAAgAAABwAZwBvAGEAbABfAHIAZQBhAGMAaABlAGQAXwBpAAAABAEAAAAEAAAAFABwAGkAYwBrAF8AYgBvAHgAXwBpAAAABQEAAAAFAAAAHABnAG8AYQBsAF8AcgBlAGEAYwBoAGUAZABfAGkAAAAGAQAAAAYAAAAUAGQAcgBvAHAAXwBiAG8AeABfAGkAAAAB= </TransitionList>
<StateList>
=AAAABQIAAAABAgAAAAICAAAABAIAAAAFAgAAAAY== </StateList>
<EventList>
=AAAABAMAAAAMAHcAbwByAGsAXwBpAwAAABwAZwBvAGEAbABfAHIAZQBhAGMAaABlAGQAXwBpAwAAABQAcABpAGMAawBfAGIAbwB4AF8AaQMAAAAUAGQAcgBvAHAAXwBiAG8AeABfAGk== </EventList>
<GraphData>
<States>
<State>
1             
<Item>
<Position>
78.941000      104.440000    
</Position>
<BasePoints>
0              0              26.163000      26.163000      -78.246560     0             
-26.187383     0             
</BasePoints>
</Item>
</State>
<State>
2             
<Item>
<Position>
234.630000     46.110000     
</Position>
<BasePoints>
0              0              39.244000      39.244000      -60            0             
-39.277675     0             
</BasePoints>
</Item>
</State>
<State>
4             
<Item>
<Position>
458.870000     43.332000     
</Position>
<BasePoints>
0              0              43.332000      43.332000      -60            0             
-43.375196     0             
</BasePoints>
</Item>
</State>
<State>
5             
<Item>
<Position>
653.450000     47.499000     
</Position>
<BasePoints>
0              0              35.974000      35.974000      -60            0             
-35.999657     0             
</BasePoints>
</Item>
</State>
<State>
6             
<Item>
<Position>
876.060000     104.440000    
</Position>
<BasePoints>
0              0              44.967500      44.967500      -60            0             
-45.014204     0             
</BasePoints>
</Item>
</State>
</States>
<TransRel>
<Trans>
1              work_i        2              +Free+       
<Item>
<Position>
78.941000      104.440000    
</Position>
<BasePoints>
71.299000      -39.580000     0              0              24.660394      -8.888043     
46.689000      -17.280000     78.769000      -29.516000     118.884953     -44.543315    
155.689000     -58.330000    
</BasePoints>
</Item>
</Trans>
<Trans>
2              goal_reached_i  4              +Free+       
<Item>
<Position>
234.630000     46.110000     
</Position>
<BasePoints>
110.070000     -9.028000      0              0              39.228753      -0.475199     
74.480000      -0.917000      126.870000     -1.574000      180.890508     -2.241975     
224.240000     -2.778000     
</BasePoints>
</Item>
</Trans>
<Trans>
4              pick_box_i    5              +Free+       
<Item>
<Position>
458.870000     43.332000     
</Position>
<BasePoints>
100.970000     -4.861000      0              0              43.310034      0.911422      
73.180000      1.560000       113.070000     2.426000       158.610460     3.398714      
194.580000     4.167000      
</BasePoints>
</Item>
</Trans>
<Trans>
5              goal_reached_i  6              +Free+       
<Item>
<Position>
653.450000     47.499000     
</Position>
<BasePoints>
106.810000     10.416000      0              0              34.915767      8.687351      
69.730000      17.703000      123.790000     31.705000      179.001732     45.804608     
222.610000     56.941000     
</BasePoints>
</Item>
</Trans>
<Trans>
6              drop_box_i    1              +Free+       
<Item>
<Position>
876.060000     104.440000    
</Position>
<BasePoints>
-417.190000    14.590000      0              0              -44.508538     6.949474      
-88.880000     13.490000      -159.650000    22.230000      -221.220000    22.230000     
-642.820000    22.230000      -642.820000    22.230000      -642.820000    22.230000     
-682.240000    22.230000      -726.790000    14.900000      -771.526130    5.422141      
-797.119000    0             
</BasePoints>
</Item>
</Trans>
</TransRel>
</GraphData>
</VioModels>
<VioLayout>
0             =AAAA/wAAAAAAAAADAAAB2AAAAdgAAAAAAQAAAAQBAAAAAQ===  0              150            950            1.004035       0             
950           
</VioLayout>
</VioSystem>
</Value>
</Variable>
<Variable>
Hs_Storage_k_TaskTransmitter  System        +Visual+     
<Value>
<VioSystem>
<Generator name="Hs_Storage_k_TaskTransmitter" ftype="System">

<Alphabet>
<Event name="tin_k">
<Controllable/>
</Event>
<Event name="tout_k">
<Controllable/>
</Event>
</Alphabet>

<StateSet>
<State id="1" name="Empty">
<Initial/>
<Marked/>
</State>
<State id="2" name="Full">
<Marked/>
</State>
</StateSet>

<TransitionRelation>
<Transition x1="1" event="tin_k" x2="2"/>
<Transition x1="2" event="tout_k" x2="1"/>
</TransitionRelation>

</Generator>
<VioModels>
<TransitionList>
=AAAAAgEAAAABAAAACgB0AGkAbgBfAGsAAAACAQAAAAIAAAAMAHQAbwB1AHQAXwBrAAAAAQ=== </TransitionList>
<StateList>
=AAAAAgIAAAABAgAAAAI== </StateList>
<EventList>
=AAAAAgMAAAAKAHQAaQBuAF8AawMAAAAMAHQAbwB1AHQAXwBr= </EventList>
<GraphData>
<States>
<State>
1             
<Item>
<Position>
89.569000      36.791000     
</Position>
<BasePoints>
0              0              36.791500      36.791500      -88.874560     0             
-36.783206     0             
</BasePoints>
</Item>
</State>
<State>
2             
<Item>
<Position>
241.410000     36.791000     
</Position>
<BasePoints>
0              0              26.163000      26.163000      -60            0             
-26.187383     0             
</BasePoints>
</Item>
</State>
</States>
<TransRel>
<Trans>
1              tin_k         2              +Free+       
<Item>
<Position>
89.569000      36.791000     
</Position>
<BasePoints>
81.241000      -7.638000      0              0              36.800627      0             
59.421000      0              88.201000      0              125.702059     0             
151.841000     0             
</BasePoints>
</Item>
</Trans>
<Trans>
2              tout_k        1              +Free+       
<Item>
<Position>
241.410000     36.791000     
</Position>
<BasePoints>
-70.600000     13.195000      0              0              -23.533698     11.482408     
-31.990000     15.263000      -41.790000     18.952000      -51.160000     20.834000     
-68.110000     24.237000      -73            23.691000      -90.050000     20.834000     
-94.530000     20.082000      -99.130000     19.037000      -117.231762    12.499404     
-151.841000    0             
</BasePoints>
</Item>
</Trans>
</TransRel>
</GraphData>
</VioModels>
<VioLayout>
0             =AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA===  0              150            472            1.217641       0             
472           
</VioLayout>
</VioSystem>
</Value>
</Variable>
<Variable>
Hs_Request_i_TaskTransmitter_Robot_i  System        +Visual+     
<Value>
<VioSystem>
<Generator name="Hs_Request_i_TaskTransmitter_Robot_i" ftype="System">

<Alphabet>
<Event name="broadcast"/>
<Event name="request_i">
<Controllable/>
</Event>
<Event name="accept_i">
<Controllable/>
</Event>
<Event name="deny_i">
<Controllable/>
</Event>
<Event name="work_i">
<Controllable/>
</Event>
<Event name="drop_box_i"/>
</Alphabet>

<StateSet>
<State id="1" name="Idle">
<Initial/>
<Marked/>
</State>
<State id="2" name="Request"/>
<State id="3" name="Wait"/>
<State id="4" name="Work"/>
<State id="5" name="WorkDone"/>
</StateSet>

<TransitionRelation>
<Transition x1="1" event="broadcast" x2="2"/>
<Transition x1="2" event="broadcast" x2="2"/>
<Transition x1="2" event="request_i" x2="3"/>
<Transition x1="3" event="accept_i" x2="4"/>
<Transition x1="3" event="deny_i" x2="1"/>
<Transition x1="4" event="broadcast" x2="4"/>
<Transition x1="4" event="work_i" x2="5"/>
<Transition x1="5" event="broadcast" x2="5"/>
<Transition x1="5" event="drop_box_i" x2="1"/>
</TransitionRelation>

</Generator>
<VioModels>
<TransitionList>
=AAAACQEAAAABAAAAEgBiAHIAbwBhAGQAYwBhAHMAdAAAAAIBAAAAAgAAABIAcgBlAHEAdQBlAHMAdABfAGkAAAADAQAAAAIAAAASAGIAcgBvAGEAZABjAGEAcwB0AAAAAgEAAAADAAAAEABhAGMAYwBlAHAAdABfAGkAAAAEAQAAAAMAAAAMAGQAZQBuAHkAXwBpAAAAAQEAAAAEAAAADAB3AG8AcgBrAF8AaQAAAAUBAAAABAAAABIAYgByAG8AYQBkAGMAYQBzAHQAAAAEAQAAAAUAAAASAGIAcgBvAGEAZABjAGEAcwB0AAAABQEAAAAFAAAAFABkAHIAbwBwAF8AYgBvAHgAXwBpAAAAAQ=== </TransitionList>
<StateList>
=AAAABQIAAAABAgAAAAICAAAAAwIAAAAEAgAAAAU== </StateList>
<EventList>
=AAAABgMAAAASAGIAcgBvAGEAZABjAGEAcwB0AwAAABIAcgBlAHEAdQBlAHMAdABfAGkDAAAAEABhAGMAYwBlAHAAdABfAGkDAAAADABkAGUAbgB5AF8AaQMAAAAMAHcAbwByAGsAXwBpAwAAABQAZAByAG8AcABfAGIAbwB4AF8AaQ=== </EventList>
<GraphData>
<States>
<State>
1             
<Item>
<Position>
81.393000      172.170000    
</Position>
<BasePoints>
0              0              28.615500      28.615500      -80.698560     0             
-28.650780     0             
</BasePoints>
</Item>
</State>
<State>
2             
<Item>
<Position>
273.850000     86.063000     
</Position>
<BasePoints>
0              0              45.785000      45.785000      -60            0             
-45.833841     0             
</BasePoints>
</Item>
</State>
<State>
3             
<Item>
<Position>
471.020000     163.840000    
</Position>
<BasePoints>
0              0              31.068500      31.068500      -60            0             
-31.082719     0             
</BasePoints>
</Item>
</State>
<State>
4             
<Item>
<Position>
648.990000     163.840000    
</Position>
<BasePoints>
0              0              33.521000      33.521000      -60            0             
-33.541239     0             
</BasePoints>
</Item>
</State>
<State>
5             
<Item>
<Position>
833.120000     213.840000    
</Position>
<BasePoints>
0              0              54.780000      54.780000      -60            0             
-54.731343     0             
</BasePoints>
</Item>
</State>
</States>
<TransRel>
<Trans>
1              broadcast     2              +Free+       
<Item>
<Position>
81.393000      172.170000    
</Position>
<BasePoints>
87.647000      -60.410000     0              0              26.317054      -11.318205    
54.757000      -24.230000     101.227000     -45.320000     150.591338     -67.389749    
192.457000     -86.107000    
</BasePoints>
</Item>
</Trans>
<Trans>
2              broadcast     2              +Free+       
<Item>
<Position>
273.850000     86.063000     
</Position>
<BasePoints>
0              -78.424100     0              0              -18.689893     -41.795761    
-19.420000     -57.662000     -13.180000     -70.785000     0              -70.785000    
8.860000       -70.785000     14.580000      -64.861000     10.057594      -44.742496    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
2              request_i     3              +Free+       
<Item>
<Position>
273.850000     86.063000     
</Position>
<BasePoints>
102.730000     22.917000      0              0              42.752786      16.556410     
76.090000      29.887000      121.910000     48.217000      168.229621     66.410037     
197.170000     77.777000     
</BasePoints>
</Item>
</Trans>
<Trans>
3              deny_i        1              +Free+       
<Item>
<Position>
471.020000     163.840000    
</Position>
<BasePoints>
-197.170000    -3.470000      0              0              -31.059822     0.639996      
-99.310000     2.110000       -267.620000    5.740000       -360.996587    7.722225      
-389.627000    8.330000      
</BasePoints>
</Item>
</Trans>
<Trans>
3              accept_i      4              +Free+       
<Item>
<Position>
471.020000     163.840000    
</Position>
<BasePoints>
90.980000      -7.640000      0              0              31.109822      0             
58.250000      0              98.800000      0              144.427457     0             
177.970000     0             
</BasePoints>
</Item>
</Trans>
<Trans>
4              broadcast     4              +Free+       
<Item>
<Position>
648.990000     163.840000    
</Position>
<BasePoints>
0              -66.160000     0              0              -14.509379     -30.229515    
-16.560000     -45.480000     -11.640000     -58.520000     0              -58.520000    
7.640000       -58.520000     12.380000      -52.900000     7.643591       -32.661226    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
4              work_i        5              +Free+       
<Item>
<Position>
648.990000     163.840000    
</Position>
<BasePoints>
81.440000      7.640000       0              0              32.432503      8.566325      
55.910000      15.030000      88.670000      24.060000      131.169517     35.608685     
184.130000     50            
</BasePoints>
</Item>
</Trans>
<Trans>
5              drop_box_i    1              +Free+       
<Item>
<Position>
833.120000     213.840000    
</Position>
<BasePoints>
-362.100000    14.580000      0              0              -53.949691     9.879944      
-90.050000     15.810000      -139.130000    22.220000      -182.740000    22.220000     
-560.660000    22.220000      -560.660000    22.220000      -560.660000    22.220000     
-616.060000    22.220000      -676.020000    -2.550000      -726.318583    -28.540735    
-751.727000    -41.670000    
</BasePoints>
</Item>
</Trans>
<Trans>
5              broadcast     5              +Free+       
<Item>
<Position>
833.120000     213.840000    
</Position>
<BasePoints>
0              -87.420000     0              0              -18.646697     -51.587203    
-18.250000     -67.500000     -12.010000     -79.780000     0              -79.780000    
8.070000       -79.780000     13.540000      -74.240000     9.843622       -53.972710    
0              0             
</BasePoints>
</Item>
</Trans>
</TransRel>
</GraphData>
</VioModels>
<VioLayout>
0             =AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAVrg===  0              150            472            1.018895       0             
472           
</VioLayout>
</VioSystem>
</Value>
</Variable>
<Variable>
E_TaskTransmitter  Alphabet      +Visual+     
<Value>
<VioAlphabet>
<E_TaskTransmitter>
tin_k         +C+           broadcast     request_i     +C+           accept_i     
+C+           tout_k        +C+           deny_i        +C+          
</E_TaskTransmitter>
<VioData>
=AAAABgAAAAoAdABpAG4AXwBrAAAAEgBiAHIAbwBhAGQAYwBhAHMAdAAAABIAcgBlAHEAdQBlAHMAdABfAGkAAAAQAGEAYwBjAGUAcAB0AF8AaQAAAAwAdABvAHUAdABfAGsAAAAMAGQAZQBuAHkAXwBp= </VioData>
<VioLayout>
0              150            150           
</VioLayout>
</VioAlphabet>
</Value>
</Variable>
<Variable>
Hsinv_Storage_k_TaskTransmitter  System        +Visual+     
<Value>
<VioSystem>
<Generator name="Hsinv_Storage_k_TaskTransmitter" ftype="System">

<Alphabet>
<Event name="tin_k">
<Controllable/>
</Event>
<Event name="broadcast"/>
<Event name="request_i">
<Controllable/>
</Event>
<Event name="accept_i">
<Controllable/>
</Event>
<Event name="tout_k">
<Controllable/>
</Event>
<Event name="deny_i">
<Controllable/>
</Event>
</Alphabet>

<StateSet>
<State id="1" name="Empty">
<Initial/>
<Marked/>
</State>
<State id="2" name="Full">
<Marked/>
</State>
</StateSet>

<TransitionRelation>
<Transition x1="1" event="tin_k" x2="2"/>
<Transition x1="1" event="broadcast" x2="1"/>
<Transition x1="1" event="request_i" x2="1"/>
<Transition x1="1" event="accept_i" x2="1"/>
<Transition x1="1" event="deny_i" x2="1"/>
<Transition x1="2" event="broadcast" x2="2"/>
<Transition x1="2" event="request_i" x2="2"/>
<Transition x1="2" event="accept_i" x2="2"/>
<Transition x1="2" event="tout_k" x2="1"/>
<Transition x1="2" event="deny_i" x2="2"/>
</TransitionRelation>

</Generator>
<VioModels>
<TransitionList>
=AAAACgEAAAABAAAACgB0AGkAbgBfAGsAAAACAQAAAAEAAAASAGIAcgBvAGEAZABjAGEAcwB0AAAAAQEAAAABAAAAEgByAGUAcQB1AGUAcwB0AF8AaQAAAAEBAAAAAQAAABAAYQBjAGMAZQBwAHQAXwBpAAAAAQEAAAABAAAADABkAGUAbgB5AF8AaQAAAAEBAAAAAgAAABIAYgByAG8AYQBkAGMAYQBzAHQAAAACAQAAAAIAAAASAHIAZQBxAHUAZQBzAHQAXwBpAAAAAgEAAAACAAAAEABhAGMAYwBlAHAAdABfAGkAAAACAQAAAAIAAAAMAHQAbwB1AHQAXwBrAAAAAQEAAAACAAAADABkAGUAbgB5AF8AaQAAAAI== </TransitionList>
<StateList>
=AAAAAgIAAAABAgAAAAI== </StateList>
<EventList>
=AAAABgMAAAAKAHQAaQBuAF8AawMAAAASAGIAcgBvAGEAZABjAGEAcwB0AwAAABIAcgBlAHEAdQBlAHMAdABfAGkDAAAAEABhAGMAYwBlAHAAdABfAGkDAAAADAB0AG8AdQB0AF8AawMAAAAMAGQAZQBuAHkAXwBp= </EventList>
<GraphData>
<States>
<State>
1             
<Item>
<Position>
0              60            
</Position>
<BasePoints>
0              0              30             30             -60            0             
-29.999314     0             
</BasePoints>
</Item>
</State>
<State>
2             
<Item>
<Position>
180            60            
</Position>
<BasePoints>
0              0              30             30             -60            0             
-29.999314     0             
</BasePoints>
</Item>
</State>
</States>
<TransRel>
<Trans>
1              tin_k         2              +Line+       
<Item>
<Position>
0              60            
</Position>
<BasePoints>
90             10             0              0              30.057993      0             
69.590039      0              109.209375     0              149.978377     0             
180            0             
</BasePoints>
</Item>
</Trans>
<Trans>
1              broadcast     1              +Spline+     
<Item>
<Position>
0              60            
</Position>
<BasePoints>
0              -80.595703     0              0              13.432348      -26.864696    
40.297852      -80.595703     -40.297852     -80.595703     -13.419461     -26.838922    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
1              request_i     1              +Spline+     
<Item>
<Position>
0              60            
</Position>
<BasePoints>
0              -80.595703     0              0              13.432348      -26.864696    
40.297852      -80.595703     -40.297852     -80.595703     -13.419461     -26.838922    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
1              accept_i      1              +Spline+     
<Item>
<Position>
0              60            
</Position>
<BasePoints>
0              -80.595703     0              0              13.432348      -26.864696    
40.297852      -80.595703     -40.297852     -80.595703     -13.419461     -26.838922    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
1              deny_i        1              +Spline+     
<Item>
<Position>
0              60            
</Position>
<BasePoints>
0              -80.595703     0              0              13.432348      -26.864696    
40.297852      -80.595703     -40.297852     -80.595703     -13.419461     -26.838922    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
2              broadcast     2              +Spline+     
<Item>
<Position>
180            60            
</Position>
<BasePoints>
0              -80.595703     0              0              13.432348      -26.864696    
40.297852      -80.595703     -40.297852     -80.595703     -13.419461     -26.838922    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
2              request_i     2              +Spline+     
<Item>
<Position>
180            60            
</Position>
<BasePoints>
0              -80.595703     0              0              13.432348      -26.864696    
40.297852      -80.595703     -40.297852     -80.595703     -13.419461     -26.838922    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
2              accept_i      2              +Spline+     
<Item>
<Position>
180            60            
</Position>
<BasePoints>
0              -80.595703     0              0              13.432348      -26.864696    
40.297852      -80.595703     -40.297852     -80.595703     -13.419461     -26.838922    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
2              tout_k        1              +Line+       
<Item>
<Position>
180            60            
</Position>
<BasePoints>
-90            -10            0              0              -30.057993     0             
-69.590039     0              -109.209375    0              -149.978377    0             
-180           0             
</BasePoints>
</Item>
</Trans>
<Trans>
2              deny_i        2              +Spline+     
<Item>
<Position>
180            60            
</Position>
<BasePoints>
0              -80.595703     0              0              13.432348      -26.864696    
40.297852      -80.595703     -40.297852     -80.595703     -13.419461     -26.838922    
0              0             
</BasePoints>
</Item>
</Trans>
</TransRel>
</GraphData>
</VioModels>
<VioLayout>
2147483648    ==  0              150            600            1              0             
300           
</VioLayout>
</VioSystem>
</Value>
</Variable>
<Variable>
Rl_Storage_k_TaskTransmitter  System        +Visual+     
<Value>
<VioSystem>
<Generator name="Rl_Storage_k_TaskTransmitter" ftype="System">

<Alphabet>
<Event name="tin_k">
<Controllable/>
</Event>
<Event name="broadcast"/>
<Event name="request_i">
<Controllable/>
</Event>
<Event name="accept_i">
<Controllable/>
</Event>
<Event name="tout_k">
<Controllable/>
</Event>
<Event name="deny_i">
<Controllable/>
</Event>
</Alphabet>

<StateSet>
<State id="1" name="Idle|Empty">
<Initial/>
<Marked/>
</State>
<State id="2" name="Broadcast|Full"/>
<State id="3" name="Reject|Empty"/>
<State id="4" name="Answer|Full"/>
<State id="5" name="Idle|Full">
<Marked/>
</State>
<State id="6" name="Reject|Full"/>
<State id="7" name="Broadcast|Empty"/>
<State id="8" name="Answer|Empty"/>
</StateSet>

<TransitionRelation>
<Transition x1="1" event="tin_k" x2="2"/>
<Transition x1="1" event="request_i" x2="3"/>
<Transition x1="2" event="broadcast" x2="2"/>
<Transition x1="2" event="request_i" x2="4"/>
<Transition x1="3" event="deny_i" x2="1"/>
<Transition x1="4" event="accept_i" x2="5"/>
<Transition x1="4" event="deny_i" x2="2"/>
<Transition x1="5" event="request_i" x2="6"/>
<Transition x1="5" event="tout_k" x2="7"/>
<Transition x1="6" event="deny_i" x2="5"/>
<Transition x1="7" event="broadcast" x2="7"/>
<Transition x1="7" event="request_i" x2="8"/>
<Transition x1="8" event="accept_i" x2="1"/>
<Transition x1="8" event="deny_i" x2="7"/>
</TransitionRelation>

</Generator>
<VioModels>
<TransitionList>
=AAAADgEAAAABAAAACgB0AGkAbgBfAGsAAAACAQAAAAEAAAASAHIAZQBxAHUAZQBzAHQAXwBpAAAAAwEAAAACAAAAEgBiAHIAbwBhAGQAYwBhAHMAdAAAAAIBAAAAAgAAABIAcgBlAHEAdQBlAHMAdABfAGkAAAAEAQAAAAMAAAAMAGQAZQBuAHkAXwBpAAAAAQEAAAAEAAAAEABhAGMAYwBlAHAAdABfAGkAAAAFAQAAAAUAAAASAHIAZQBxAHUAZQBzAHQAXwBpAAAABgEAAAAFAAAADAB0AG8AdQB0AF8AawAAAAcBAAAABgAAAAwAZABlAG4AeQBfAGkAAAAFAQAAAAcAAAASAGIAcgBvAGEAZABjAGEAcwB0AAAABwEAAAAHAAAAEgByAGUAcQB1AGUAcwB0AF8AaQAAAAgBAAAACAAAABAAYQBjAGMAZQBwAHQAXwBpAAAAAQEAAAAEAAAADABkAGUAbgB5AF8AaQAAAAIBAAAACAAAAAwAZABlAG4AeQBfAGkAAAAH= </TransitionList>
<StateList>
=AAAACAIAAAABAgAAAAICAAAAAwIAAAAEAgAAAAUCAAAABgIAAAAHAgAAAAg== </StateList>
<EventList>
=AAAABgMAAAAKAHQAaQBuAF8AawMAAAASAGIAcgBvAGEAZABjAGEAcwB0AwAAABIAcgBlAHEAdQBlAHMAdABfAGkDAAAAEABhAGMAYwBlAHAAdABfAGkDAAAADAB0AG8AdQB0AF8AawMAAAAMAGQAZQBuAHkAXwBp= </EventList>
<GraphData>
<States>
<State>
1             
<Item>
<Position>
109.190000     273.650000    
</Position>
<BasePoints>
0              0              56.415000      56.415000      -108.495560    0             
-56.480777     0             
</BasePoints>
</Item>
</State>
<State>
5             
<Item>
<Position>
805.290000     102.820000    
</Position>
<BasePoints>
0              0              44.967500      44.967500      -60            0             
-45.014591     0             
</BasePoints>
</Item>
</State>
<State>
2             
<Item>
<Position>
349.810000     111.150000    
</Position>
<BasePoints>
0              0              70.315000      70.315000      -60            0             
-70.360909     0             
</BasePoints>
</Item>
</State>
<State>
3             
<Item>
<Position>
349.810000     273.650000    
</Position>
<BasePoints>
0              0              67.040000      67.040000      -60            0             
-67.082863     0             
</BasePoints>
</Item>
</State>
<State>
4             
<Item>
<Position>
593.690000     102.820000    
</Position>
<BasePoints>
0              0              59.685000      59.685000      -60            0             
-59.648724     0             
</BasePoints>
</Item>
</State>
<State>
6             
<Item>
<Position>
1045.900000    55.596000     
</Position>
<BasePoints>
0              0              55.595000      55.595000      -60            0             
-55.551167     0             
</BasePoints>
</Item>
</State>
<State>
7             
<Item>
<Position>
1045.900000    258.370000    
</Position>
<BasePoints>
0              0              81.760000      81.760000      -60            0             
-81.834068     0             
</BasePoints>
</Item>
</State>
<State>
8             
<Item>
<Position>
1312.700000    305.600000    
</Position>
<BasePoints>
0              0              71.130000      71.130000      -60            0             
-71.063347     0             
</BasePoints>
</Item>
</State>
</States>
<TransRel>
<Trans>
1              tin_k         2              +Free+       
<Item>
<Position>
109.190000     273.650000    
</Position>
<BasePoints>
113.360000     -102.080000    0              0              47.111122      -31.237914    
81.990000      -55.070000     130.310000     -88.090000     182.228495     -123.382720   
240.620000     -162.500000   
</BasePoints>
</Item>
</Trans>
<Trans>
1              request_i     3              +Free+       
<Item>
<Position>
109.190000     273.650000    
</Position>
<BasePoints>
113.360000     -7.640000      0              0              56.494452      0             
87.050000      0              125.490000     0              173.485452     0             
240.620000     0             
</BasePoints>
</Item>
</Trans>
<Trans>
5              request_i     6              +Free+       
<Item>
<Position>
805.290000     102.820000    
</Position>
<BasePoints>
101.910000     -35.418000     0              0              43.644632      -10.691088    
52.390000      -12.784000     61.450000      -14.868000     69.960000      -16.668000    
103.400000     -23.744000     140.780000     -30.601000     185.674554     -38.267412    
240.610000     -47.224000    
</BasePoints>
</Item>
</Trans>
<Trans>
5              tout_k        7              +Free+       
<Item>
<Position>
805.290000     102.820000    
</Position>
<BasePoints>
101.910000     39.580000      0              0              38.023427      23.968075     
70.890000      45.460000      118.870000     76.840000      171.748423     111.284278    
240.610000     155.550000    
</BasePoints>
</Item>
</Trans>
<Trans>
2              broadcast     2              +Free+       
<Item>
<Position>
349.810000     111.150000    
</Position>
<BasePoints>
0              -102.949900    0              0              -22.944876     -66.414877    
-21.370000     -83.149000     -13.680000     -95.311000     0              -95.311000    
9.400000       -95.311000     15.970000      -89.562000     16.720651      -68.425720    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
2              request_i     4              +Free+       
<Item>
<Position>
349.810000     111.150000    
</Position>
<BasePoints>
127.250000     -11.804000     0              0              70.410769      -2.387652     
101.610000     -3.460000      138.380000     -4.730000      184.176705     -6.303834     
243.880000     -8.330000     
</BasePoints>
</Item>
</Trans>
<Trans>
3              deny_i        1              +Free+       
<Item>
<Position>
349.810000     273.650000    
</Position>
<BasePoints>
-127.260000    13.200000      0              0              -65.185906     16.104166     
-75.250000     18.080000      -85.520000     19.760000      -95.320000     20.830000     
-123.540000    23.930000      -131.040000    24.440000      -159.210000    20.830000     
-163.480000    20.290000      -167.860000    19.580000      -186.098143    14.960217     
-240.620000    0             
</BasePoints>
</Item>
</Trans>
<Trans>
4              accept_i      5              +Free+       
<Item>
<Position>
593.690000     102.820000    
</Position>
<BasePoints>
113.160000     -7.641000      0              0              59.640792      0             
89.010000      0              123.720000     0              166.668843     0             
211.600000     0             
</BasePoints>
</Item>
</Trans>
<Trans>
4              deny_i        2              +Free+       
<Item>
<Position>
593.690000     102.820000    
</Position>
<BasePoints>
-116.630000    18.750000      0              0              -56.324619     20.000335     
-65.710000     22.660000      -75.390000     24.950000      -84.680000     26.390000     
-109.400000    30.210000      -136.410000    29.060000      -175.021544    22.983917     
-243.880000    8.330000      
</BasePoints>
</Item>
</Trans>
<Trans>
6              deny_i        5              +Free+       
<Item>
<Position>
1045.900000    55.596000     
</Position>
<BasePoints>
-138.700000    40.972000      0              0              -48.545046     27.030129     
-66.290000     35.761000      -86.880000     44.248000      -106.760000    48.614000     
-130.940000    53.924000      -158.210000    54.514000      -195.814828    51.945283     
-240.610000    47.224000     
</BasePoints>
</Item>
</Trans>
<Trans>
7              broadcast     7              +Free+       
<Item>
<Position>
1045.900000    258.370000    
</Position>
<BasePoints>
0              -114.390000    0              0              -24.675738     -78.123185    
-22.200000     -94.780000     -14.000000     -106.760000    0              -106.760000   
9.600000       -106.760000    16.500000      -101.100000    17.913056      -79.760695    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
7              request_i     8              +Free+       
<Item>
<Position>
1045.900000    258.370000    
</Position>
<BasePoints>
138.700000     10.420000      0              0              80.678557      14.183811     
112.700000     19.920000      149.900000     26.570000      196.727481     34.909192     
266.800000     47.230000     
</BasePoints>
</Item>
</Trans>
<Trans>
8              accept_i      1              +Free+       
<Item>
<Position>
1312.700000    305.600000    
</Position>
<BasePoints>
-605.850000    65.970000      0              0              -65.378322     28.320655     
-118.300000    49.170000      -195.200000    73.610000      -265.400000    73.610000     
-964.280000    73.610000      -964.280000    73.610000      -964.280000    73.610000     
-1030.840000   73.610000      -1099.570000   38.640000      -1157.471986   0.753858      
-1203.510000   -31.950000    
</BasePoints>
</Item>
</Trans>
<Trans>
8              deny_i        7              +Free+       
<Item>
<Position>
1312.700000    305.600000    
</Position>
<BasePoints>
-128.100000    -6.250000      0              0              -70.869006     7.287095      
-98.800000     8.500000       -131.300000    7.700000       -160.000000    1.380000      
-166.900000    -0.120000      -173.800000    -2.100000      -193.636070    -10.851976    
-266.800000    -47.230000    
</BasePoints>
</Item>
</Trans>
</TransRel>
</GraphData>
</VioModels>
<VioLayout>
0             =AAAA/wAAAAAAAAADAAACPgAABT4AAAAAAQAAAAQBAAAAAQ===  0              150            1342           0.945544       0             
574           
</VioLayout>
</VioSystem>
</Value>
</Variable>
<Variable>
Gl_TaskTrasmitter_Robot_i  System        +Visual+     
<Value>
<VioSystem>
<Generator name="Gl_TaskTrasmitter_Robot_i" ftype="System">

<Alphabet>
<Event name="tin_k">
<Controllable/>
</Event>
<Event name="broadcast"/>
<Event name="request_i">
<Controllable/>
</Event>
<Event name="accept_i">
<Controllable/>
</Event>
<Event name="tout_k">
<Controllable/>
</Event>
<Event name="deny_i">
<Controllable/>
</Event>
<Event name="work_i">
<Controllable/>
</Event>
<Event name="goal_reached_i"/>
<Event name="pick_box_i"/>
<Event name="drop_box_i"/>
</Alphabet>

<StateSet>
<State id="1" name="Idle|Idle">
<Initial/>
<Marked/>
</State>
<State id="2" name="Broadcast|Idle"/>
<State id="3" name="Reject|Idle"/>
<State id="4" name="Idle|ToStart"/>
<State id="5" name="Broadcast|ToStart"/>
<State id="6" name="Reject|ToStart"/>
<State id="7" name="Idle|PickBox"/>
<State id="8" name="Broadcast|PickBox"/>
<State id="9" name="Reject|PickBox"/>
<State id="10" name="Idle|ToEnd"/>
<State id="11" name="Broadcast|ToEnd"/>
<State id="12" name="Reject|ToEnd"/>
<State id="13" name="Idle|DropBox"/>
<State id="14" name="Broadcast|DropBox"/>
<State id="15" name="Reject|DropBox"/>
<State id="16" name="Answer|DropBox"/>
<State id="17" name="Answer|Idle"/>
<State id="18" name="Answer|ToStart"/>
<State id="19" name="Answer|PickBox"/>
<State id="20" name="Answer|ToEnd"/>
</StateSet>

<TransitionRelation>
<Transition x1="1" event="tin_k" x2="2"/>
<Transition x1="1" event="request_i" x2="3"/>
<Transition x1="1" event="tout_k" x2="2"/>
<Transition x1="1" event="work_i" x2="4"/>
<Transition x1="2" event="broadcast" x2="2"/>
<Transition x1="2" event="request_i" x2="17"/>
<Transition x1="2" event="work_i" x2="5"/>
<Transition x1="3" event="deny_i" x2="1"/>
<Transition x1="3" event="work_i" x2="6"/>
<Transition x1="4" event="tin_k" x2="5"/>
<Transition x1="4" event="request_i" x2="6"/>
<Transition x1="4" event="tout_k" x2="5"/>
<Transition x1="4" event="goal_reached_i" x2="7"/>
<Transition x1="5" event="broadcast" x2="5"/>
<Transition x1="5" event="request_i" x2="18"/>
<Transition x1="5" event="goal_reached_i" x2="8"/>
<Transition x1="6" event="deny_i" x2="4"/>
<Transition x1="6" event="goal_reached_i" x2="9"/>
<Transition x1="7" event="tin_k" x2="8"/>
<Transition x1="7" event="request_i" x2="9"/>
<Transition x1="7" event="tout_k" x2="8"/>
<Transition x1="7" event="pick_box_i" x2="10"/>
<Transition x1="8" event="broadcast" x2="8"/>
<Transition x1="8" event="request_i" x2="19"/>
<Transition x1="8" event="pick_box_i" x2="11"/>
<Transition x1="9" event="deny_i" x2="7"/>
<Transition x1="9" event="pick_box_i" x2="12"/>
<Transition x1="10" event="tin_k" x2="11"/>
<Transition x1="10" event="request_i" x2="12"/>
<Transition x1="10" event="tout_k" x2="11"/>
<Transition x1="10" event="goal_reached_i" x2="13"/>
<Transition x1="11" event="broadcast" x2="11"/>
<Transition x1="11" event="request_i" x2="20"/>
<Transition x1="11" event="goal_reached_i" x2="14"/>
<Transition x1="12" event="deny_i" x2="10"/>
<Transition x1="12" event="goal_reached_i" x2="15"/>
<Transition x1="13" event="tin_k" x2="14"/>
<Transition x1="13" event="request_i" x2="15"/>
<Transition x1="13" event="tout_k" x2="14"/>
<Transition x1="13" event="drop_box_i" x2="1"/>
<Transition x1="14" event="broadcast" x2="14"/>
<Transition x1="14" event="request_i" x2="16"/>
<Transition x1="14" event="drop_box_i" x2="2"/>
<Transition x1="15" event="deny_i" x2="13"/>
<Transition x1="15" event="drop_box_i" x2="3"/>
<Transition x1="16" event="accept_i" x2="13"/>
<Transition x1="16" event="deny_i" x2="14"/>
<Transition x1="16" event="drop_box_i" x2="17"/>
<Transition x1="17" event="accept_i" x2="1"/>
<Transition x1="17" event="deny_i" x2="2"/>
<Transition x1="17" event="work_i" x2="18"/>
<Transition x1="18" event="accept_i" x2="4"/>
<Transition x1="18" event="deny_i" x2="5"/>
<Transition x1="18" event="goal_reached_i" x2="19"/>
<Transition x1="19" event="accept_i" x2="7"/>
<Transition x1="19" event="deny_i" x2="8"/>
<Transition x1="19" event="pick_box_i" x2="20"/>
<Transition x1="20" event="accept_i" x2="10"/>
<Transition x1="20" event="deny_i" x2="11"/>
<Transition x1="20" event="goal_reached_i" x2="16"/>
</TransitionRelation>

</Generator>
<VioModels>
<TransitionList>
=AAAAPAEAAAABAAAACgB0AGkAbgBfAGsAAAACAQAAAAEAAAASAHIAZQBxAHUAZQBzAHQAXwBpAAAAAwEAAAABAAAADAB0AG8AdQB0AF8AawAAAAIBAAAAAQAAAAwAdwBvAHIAawBfAGkAAAAEAQAAAAIAAAASAGIAcgBvAGEAZABjAGEAcwB0AAAAAgEAAAACAAAAEgByAGUAcQB1AGUAcwB0AF8AaQAAABEBAAAAAgAAAAwAdwBvAHIAawBfAGkAAAAFAQAAAAMAAAAMAGQAZQBuAHkAXwBpAAAAAQEAAAADAAAADAB3AG8AcgBrAF8AaQAAAAYBAAAABAAAAAoAdABpAG4AXwBrAAAABQEAAAAEAAAAEgByAGUAcQB1AGUAcwB0AF8AaQAAAAYBAAAABAAAAAwAdABvAHUAdABfAGsAAAAFAQAAAAQAAAAcAGcAbwBhAGwAXwByAGUAYQBjAGgAZQBkAF8AaQAAAAcBAAAABQAAABIAYgByAG8AYQBkAGMAYQBzAHQAAAAFAQAAAAUAAAASAHIAZQBxAHUAZQBzAHQAXwBpAAAAEgEAAAAFAAAAHABnAG8AYQBsAF8AcgBlAGEAYwBoAGUAZABfAGkAAAAIAQAAAAYAAAAMAGQAZQBuAHkAXwBpAAAABAEAAAAGAAAAHABnAG8AYQBsAF8AcgBlAGEAYwBoAGUAZABfAGkAAAAJAQAAAAcAAAAKAHQAaQBuAF8AawAAAAgBAAAABwAAABIAcgBlAHEAdQBlAHMAdABfAGkAAAAJAQAAAAcAAAAMAHQAbwB1AHQAXwBrAAAACAEAAAAHAAAAFABwAGkAYwBrAF8AYgBvAHgAXwBpAAAACgEAAAAIAAAAEgBiAHIAbwBhAGQAYwBhAHMAdAAAAAgBAAAACAAAABIAcgBlAHEAdQBlAHMAdABfAGkAAAATAQAAAAgAAAAUAHAAaQBjAGsAXwBiAG8AeABfAGkAAAALAQAAAAkAAAAMAGQAZQBuAHkAXwBpAAAABwEAAAAJAAAAFABwAGkAYwBrAF8AYgBvAHgAXwBpAAAADAEAAAAKAAAACgB0AGkAbgBfAGsAAAALAQAAAAoAAAASAHIAZQBxAHUAZQBzAHQAXwBpAAAADAEAAAAKAAAADAB0AG8AdQB0AF8AawAAAAsBAAAACgAAABwAZwBvAGEAbABfAHIAZQBhAGMAaABlAGQAXwBpAAAADQEAAAALAAAAEgBiAHIAbwBhAGQAYwBhAHMAdAAAAAsBAAAACwAAABIAcgBlAHEAdQBlAHMAdABfAGkAAAAUAQAAAAsAAAAcAGcAbwBhAGwAXwByAGUAYQBjAGgAZQBkAF8AaQAAAA4BAAAADAAAAAwAZABlAG4AeQBfAGkAAAAKAQAAAAwAAAAcAGcAbwBhAGwAXwByAGUAYQBjAGgAZQBkAF8AaQAAAA8BAAAADQAAAAoAdABpAG4AXwBrAAAADgEAAAANAAAAEgByAGUAcQB1AGUAcwB0AF8AaQAAAA8BAAAADQAAAAwAdABvAHUAdABfAGsAAAAOAQAAAA0AAAAUAGQAcgBvAHAAXwBiAG8AeABfAGkAAAABAQAAAA4AAAASAGIAcgBvAGEAZABjAGEAcwB0AAAADgEAAAAOAAAAEgByAGUAcQB1AGUAcwB0AF8AaQAAABABAAAADgAAABQAZAByAG8AcABfAGIAbwB4AF8AaQAAAAIBAAAADwAAAAwAZABlAG4AeQBfAGkAAAANAQAAAA8AAAAUAGQAcgBvAHAAXwBiAG8AeABfAGkAAAADAQAAABAAAAAQAGEAYwBjAGUAcAB0AF8AaQAAAA0BAAAAEAAAABQAZAByAG8AcABfAGIAbwB4AF8AaQAAABEBAAAAEQAAABAAYQBjAGMAZQBwAHQAXwBpAAAAAQEAAAARAAAADAB3AG8AcgBrAF8AaQAAABIBAAAAEgAAABAAYQBjAGMAZQBwAHQAXwBpAAAABAEAAAASAAAAHABnAG8AYQBsAF8AcgBlAGEAYwBoAGUAZABfAGkAAAATAQAAABMAAAAQAGEAYwBjAGUAcAB0AF8AaQAAAAcBAAAAEwAAABQAcABpAGMAawBfAGIAbwB4AF8AaQAAABQBAAAAFAAAABAAYQBjAGMAZQBwAHQAXwBpAAAACgEAAAAUAAAAHABnAG8AYQBsAF8AcgBlAGEAYwBoAGUAZABfAGkAAAAQAQAAABAAAAAMAGQAZQBuAHkAXwBpAAAADgEAAAARAAAADABkAGUAbgB5AF8AaQAAAAIBAAAAEgAAAAwAZABlAG4AeQBfAGkAAAAFAQAAABMAAAAMAGQAZQBuAHkAXwBpAAAACAEAAAAUAAAADABkAGUAbgB5AF8AaQAAAAs== </TransitionList>
<StateList>
=AAAAFAIAAAABAgAAAAICAAAAAwIAAAAEAgAAAAUCAAAABgIAAAAHAgAAAAgCAAAACQIAAAAKAgAAAAsCAAAADAIAAAANAgAAAA4CAAAADwIAAAAQAgAAABECAAAAEgIAAAATAgAAABQ== </StateList>
<EventList>
=AAAACgMAAAAKAHQAaQBuAF8AawMAAAASAGIAcgBvAGEAZABjAGEAcwB0AwAAABIAcgBlAHEAdQBlAHMAdABfAGkDAAAAEABhAGMAYwBlAHAAdABfAGkDAAAADAB0AG8AdQB0AF8AawMAAAAMAGQAZQBuAHkAXwBpAwAAAAwAdwBvAHIAawBfAGkDAAAAHABnAG8AYQBsAF8AcgBlAGEAYwBoAGUAZABfAGkDAAAAFABwAGkAYwBrAF8AYgBvAHgAXwBpAwAAABQAZAByAG8AcABfAGIAbwB4AF8AaQ=== </EventList>
<GraphData>
<States>
<State>
1             
<Item>
<Position>
93.657000      200           
</Position>
<BasePoints>
0              0              40.879500      40.879500      -92.962560     0             
-40.876257     0             
</BasePoints>
</Item>
</State>
<State>
2             
<Item>
<Position>
283.110000     365.280000    
</Position>
<BasePoints>
0              0              59.685000      59.685000      -60            0             
-59.647530     0             
</BasePoints>
</Item>
</State>
<State>
3             
<Item>
<Position>
2329.800000    68.056000     
</Position>
<BasePoints>
0              0              51.510000      51.510000      -60            0             
-51.603467     0             
</BasePoints>
</Item>
</State>
<State>
4             
<Item>
<Position>
2821.700000    258.330000    
</Position>
<BasePoints>
0              0              53.960000      53.960000      -60            0             
-53.911064     0             
</BasePoints>
</Item>
</State>
<State>
5             
<Item>
<Position>
505.840000     431.940000    
</Position>
<BasePoints>
0              0              72.765000      72.765000      -60            0             
-72.700914     0             
</BasePoints>
</Item>
</State>
<State>
6             
<Item>
<Position>
2539.400000    68.056000     
</Position>
<BasePoints>
0              0              64.590000      64.590000      -60            0             
-64.695131     0             
</BasePoints>
</Item>
</State>
<State>
7             
<Item>
<Position>
3101.500000    602.780000    
</Position>
<BasePoints>
0              0              58.050000      58.050000      -60            0             
-58.008540     0             
</BasePoints>
</Item>
</State>
<State>
8             
<Item>
<Position>
797.120000     606.940000    
</Position>
<BasePoints>
0              0              76.855000      76.855000      -60            0             
-76.915460     0             
</BasePoints>
</Item>
</State>
<State>
9             
<Item>
<Position>
3350.900000    602.780000    
</Position>
<BasePoints>
0              0              68.675000      68.675000      -60            0             
-68.720510     0             
</BasePoints>
</Item>
</State>
<State>
10            
<Item>
<Position>
3350.900000    919.440000    
</Position>
<BasePoints>
0              0              50.690000      50.690000      -60            0             
-50.781568     0             
</BasePoints>
</Item>
</State>
<State>
11            
<Item>
<Position>
1060.100000    1061.100000   
</Position>
<BasePoints>
0              0              69.495000      69.495000      -60            0             
-69.626521     0             
</BasePoints>
</Item>
</State>
<State>
12            
<Item>
<Position>
3596.200000    919.440000    
</Position>
<BasePoints>
0              0              61.320000      61.320000      -60            0             
-61.407539     0             
</BasePoints>
</Item>
</State>
<State>
13            
<Item>
<Position>
1855.700000    812.500000    
</Position>
<BasePoints>
0              0              59.685000      59.685000      -60            0             
-59.647530     0             
</BasePoints>
</Item>
</State>
<State>
14            
<Item>
<Position>
1349           783.330000    
</Position>
<BasePoints>
0              0              77.670000      77.670000      -60            0             
-77.617885     0             
</BasePoints>
</Item>
</State>
<State>
15            
<Item>
<Position>
2091.300000    780.560000    
</Position>
<BasePoints>
0              0              70.315000      70.315000      -60            0             
-70.359501     0             
</BasePoints>
</Item>
</State>
<State>
16            
<Item>
<Position>
1605.800000    669.440000    
</Position>
<BasePoints>
0              0              73.585000      73.585000      -60            0             
-73.637481     0             
</BasePoints>
</Item>
</State>
<State>
17            
<Item>
<Position>
1855.700000    238.890000    
</Position>
<BasePoints>
0              0              55.595000      55.595000      -60            0             
-55.550055     0             
</BasePoints>
</Item>
</State>
<State>
18            
<Item>
<Position>
2539.400000    344.440000    
</Position>
<BasePoints>
0              0              67.860000      67.860000      -60            0             
-67.982725     0             
</BasePoints>
</Item>
</State>
<State>
19            
<Item>
<Position>
2821.700000    615.280000    
</Position>
<BasePoints>
0              0              72.765000      72.765000      -60            0             
-72.700914     0             
</BasePoints>
</Item>
</State>
<State>
20            
<Item>
<Position>
3101.500000    838.890000    
</Position>
<BasePoints>
0              0              65.405000      65.405000      -60            0             
-65.442530     0             
</BasePoints>
</Item>
</State>
</States>
<TransRel>
<Trans>
1              tin_k         2              +Free+       
<Item>
<Position>
93.657000      200           
</Position>
<BasePoints>
85.323000      53.470000      0              0              30.053058      27.792920     
41.563000      38.750000      54.183000      50.590000      65.883000      61.110000     
87.243000      80.310000      111.203000     100.930000     143.374274     127.386505    
189.453000     165.280000    
</BasePoints>
</Item>
</Trans>
<Trans>
1              tout_k        2              +Free+       
<Item>
<Position>
93.657000      200           
</Position>
<BasePoints>
85.323000      106.250000     0              0              15.347493      37.967423     
26.033000      62.490000      42.783000      93.350000      65.883000      113.890000    
80.823000      127.180000     99.703000      137.370000     132.395302     147.536486    
189.453000     165.280000    
</BasePoints>
</Item>
</Trans>
<Trans>
1              request_i     3              +Free+       
<Item>
<Position>
93.657000      200           
</Position>
<BasePoints>
1106.843000    -139.583000    0              0              17.366077      -36.978443    
28.333000      -58.170000     44.543000      -83.090000     65.883000      -98.610000    
111.393000     -131.727000    131.773000     -131.944000    188.063000     -131.944000   
188.063000     -131.944000    188.063000     -131.944000    1999.043000    -131.944000   
2057.043000    -131.944000    2123.143000    -131.944000    2184.608968    -131.944000   
2236.143000    -131.944000   
</BasePoints>
</Item>
</Trans>
<Trans>
1              work_i        4              +Free+       
<Item>
<Position>
93.657000      200           
</Position>
<BasePoints>
1385.743000    -86.810000     0              0              33.690459      -23.092070    
69.953000      -46.680000     130.333000     -79.170000     188.063000     -79.170000    
188.063000     -79.170000     188.063000     -79.170000     1387.143000    -79.170000    
1941.943000    -79.170000     2083.043000    -56.850000     2630.243000    34.720000     
2640.543000    36.440000      2651.343000    38.670000      2675.686411    44.909784     
2728.043000    58.330000     
</BasePoints>
</Item>
</Trans>
<Trans>
2              broadcast     2              +Free+       
<Item>
<Position>
283.110000     365.280000    
</Position>
<BasePoints>
0              -92.320000     0              0              -19.142229     -56.488538    
-18.340000     -72.490000     -11.940000     -84.690000     0              -84.690000    
8.020000       -84.690000     13.540000      -79.180000     10.062413      -58.843562    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
2              work_i        5              +Free+       
<Item>
<Position>
283.110000     365.280000    
</Position>
<BasePoints>
104.820000     17.360000      0              0              57.234729      16.923391     
82.280000      24.520000      111.980000     33.520000      152.970678     45.785743     
222.730000     66.660000     
</BasePoints>
</Item>
</Trans>
<Trans>
2              request_i     17             +Free+       
<Item>
<Position>
283.110000     365.280000    
</Position>
<BasePoints>
776.990000     -92.360000     0              0              51.038000      -31.156167    
94.560000      -55.440000     159.750000     -84.720000     221.340000     -84.720000    
221.340000     -84.720000     221.340000     -84.720000     1324.090000    -84.720000    
1385.890000    -84.720000     1455.090000    -97.540000     1518.544744    -113.120165   
1572.590000    -126.390000   
</BasePoints>
</Item>
</Trans>
<Trans>
3              deny_i        1              +Free+       
<Item>
<Position>
2329.800000    68.056000     
</Position>
<BasePoints>
-1129.300000   -60.417100     0              0              -48.616680     -17.001948    
-96.200000     -32.547000     -170.700000    -52.778000     -237.100000    -52.778000    
-2048.080000   -52.778000     -2048.080000   -52.778000     -2048.080000   -52.778000    
-2107.730000   -52.778000     -2127.320000   -38.631000     -2170.260000   2.777000      
-2192.080000   23.816000      -2207.680000   53.794000      -2222.161703   93.555975     
-2236.143000   131.944000    
</BasePoints>
</Item>
</Trans>
<Trans>
3              work_i        6              +Free+       
<Item>
<Position>
2329.800000    68.056000     
</Position>
<BasePoints>
96.600000      -7.639000      0              0              51.493994      0             
75.700000      0              104.500000     0              145.041688     0             
209.600000     0             
</BasePoints>
</Item>
</Trans>
<Trans>
4              tin_k         5              +Free+       
<Item>
<Position>
2821.700000    258.330000    
</Position>
<BasePoints>
-1084          185.420000     0              0              -15.676398     51.593213     
-29.800000     92.180000      -55.400000     145.540000     -97.800000     173.610000    
-166           218.810000     -199.100000    193.060000     -280.900000    193.060000    
-2025.970000   193.060000     -2025.970000   193.060000     -2025.970000   193.060000    
-2094.460000   193.060000     -2171.900000   187.540000     -2243.458449   180.615791    
-2315.860000   173.610000    
</BasePoints>
</Item>
</Trans>
<Trans>
4              tout_k        5              +Free+       
<Item>
<Position>
2821.700000    258.330000    
</Position>
<BasePoints>
-1084          77.090000      0              0              -53.881756     -1.968374     
-122.600000    -3.910000      -245.600000    -5.170000      -350.200000    5.560000      
-448.400000    15.640000      -470.500000    34.950000      -568.500000    47.230000     
-1297.300000   138.490000     -1493.400000   27.370000      -2218.100000   147.230000    
-2222.630000   147.980000     -2227.260000   148.850000     -2245.689693   154.000330    
-2315.860000   173.610000    
</BasePoints>
</Item>
</Trans>
<Trans>
4              request_i     6              +Free+       
<Item>
<Position>
2821.700000    258.330000    
</Position>
<BasePoints>
-143.600000    -182.636000    0              0              -21.473172     -49.477767    
-37.500000     -82.730000     -62.900000     -124.220000    -97.800000     -148.610000   
-115.100000    -160.726000    -162           -171.269000    -218.508288    -180.196182   
-282.300000    -190.274000   
</BasePoints>
</Item>
</Trans>
<Trans>
4              goal_reached_i  7              +Free+       
<Item>
<Position>
2821.700000    258.330000    
</Position>
<BasePoints>
143.600000     106.250000     0              0              35.864121      40.389593     
74.100000      85.350000      137.100000     160.290000     189.400000     226.390000    
204.900000     245.910000     221.400000     267.580000     244.623025     298.147704    
279.800000     344.450000    
</BasePoints>
</Item>
</Trans>
<Trans>
5              broadcast     5              +Free+       
<Item>
<Position>
505.840000     431.940000    
</Position>
<BasePoints>
0              -105.400000    0              0              -21.316102     -69.586115    
-19.550000     -85.940000     -12.420000     -97.760000     0              -97.760000    
8.530000       -97.760000     14.570000      -92.180000     11.368551      -71.925400    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
5              goal_reached_i  8              +Free+       
<Item>
<Position>
505.840000     431.940000    
</Position>
<BasePoints>
143.590000     53.480000      0              0              62.550555      37.110811     
106.680000     63.870000      165.760000     99.710000      225.262702     135.401192    
291.280000     175           
</BasePoints>
</Item>
</Trans>
<Trans>
5              request_i     18             +Free+       
<Item>
<Position>
505.840000     431.940000    
</Position>
<BasePoints>
1099.960000    -59.020000     0              0              72.640958      -3.061914     
384.840000     -16.520000     1621.960000    -69.820000     1965.630116    -84.582118    
2033.560000    -87.500000    
</BasePoints>
</Item>
</Trans>
<Trans>
6              deny_i        4              +Free+       
<Item>
<Position>
2539.400000    68.056000     
</Position>
<BasePoints>
138.700000     52.084000      0              0              52.887628      37.045199     
65.800000      45.354000      79.500000      53.374000      92.900000      59.724000     
131.500000     78.054000      148.300000     66.204000      184.500000     88.884000     
204.800000     101.584000     223.800000     119.534000     247.864938     148.634064    
282.300000     190.274000    
</BasePoints>
</Item>
</Trans>
<Trans>
6              goal_reached_i  9              +Free+       
<Item>
<Position>
2539.400000    68.056000     
</Position>
<BasePoints>
425.900000     88.194000      0              0              63.350657      -12.400193    
99.800000      -17.123000     145.800000     -18.574000     184.500000     -5.556000     
446.700000     82.524000      668.700000     344.214000     770.240805     479.679957    
811.500000     534.724000    
</BasePoints>
</Item>
</Trans>
<Trans>
7              tin_k         8              +Free+       
<Item>
<Position>
3101.500000    602.780000    
</Position>
<BasePoints>
-1133.300000   -106.250000    0              0              -50.302017     -28.946071    
-104.500000    -58.140000     -194.700000    -98.610000     -278.400000    -98.610000    
-2042.800000   -98.610000     -2042.800000   -98.610000     -2042.800000   -98.610000    
-2107.910000   -98.610000     -2176.230000   -70.400000     -2237.927407   -34.503327    
-2304.380000   4.160000      
</BasePoints>
</Item>
</Trans>
<Trans>
7              tout_k        8              +Free+       
<Item>
<Position>
3101.500000    602.780000    
</Position>
<BasePoints>
-1133.300000   324.300000     0              0              -26.521362     51.726625     
-41.700000     77.360000      -63.200000     106.250000     -90.400000     123.610000    
-370.100000    302.130000     -1239.500000   365.580000     -1569.300000   329.160000    
-1633.600000   322.060000     -1885.900000   267.550000     -1946.900000   240.280000    
-2053.700000   192.480000     -2164.040000   113.960000     -2243.763537   51.585450     
-2304.380000   4.160000      
</BasePoints>
</Item>
</Trans>
<Trans>
7              request_i     9              +Free+       
<Item>
<Position>
3101.500000    602.780000    
</Position>
<BasePoints>
123.100000     -7.640000      0              0              58.027242      0             
90.600000      0              131.100000     0              180.661427     0             
249.400000     0             
</BasePoints>
</Item>
</Trans>
<Trans>
7              pick_box_i    10             +Free+       
<Item>
<Position>
3101.500000    602.780000    
</Position>
<BasePoints>
123.100000     111.800000     0              0              36.484509      45.103603     
82.200000      103.690000     160.800000     204.680000     217.899977     276.847668    
249.400000     316.660000    
</BasePoints>
</Item>
</Trans>
<Trans>
8              broadcast     8              +Free+       
<Item>
<Position>
797.120000     606.940000    
</Position>
<BasePoints>
0              -109.490000    0              0              -24.317895     -73.023479    
-22.190000     -89.760000     -14.060000     -101.850000    0              -101.850000   
9.670000       -101.850000    16.530000      -96.140000     13.027023      -75.766365    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
8              pick_box_i    11             +Free+       
<Item>
<Position>
797.120000     606.940000    
</Position>
<BasePoints>
135.190000     343.750000     0              0              47.915834      60.207593     
59.150000      77.530000      69.930000      97.110000      76.850000      116.670000    
111.850000     215.570000     46.780000      262.100000     101.850000     351.390000    
121.260000     382.850000     154.290000     406.460000     199.214328     426.175617    
262.980000     454.160000    
</BasePoints>
</Item>
</Trans>
<Trans>
8              request_i     19             +Free+       
<Item>
<Position>
797.120000     606.940000    
</Position>
<BasePoints>
1058.580000    -57.630000     0              0              73.930789      -21.576025    
125.940000     -35.150000     197.450000     -50            261.580000     -50           
261.580000     -50            261.580000     -50            810.080000     -50           
969.880000     -50            1009.880000    -48.610000     1169.680000    -48.610000    
1169.680000    -48.610000     1169.680000    -48.610000     1743.680000    -48.610000    
1811.280000    -48.610000     1886.180000    -32.430000     1954.704419    -12.244013    
2024.580000    8.340000      
</BasePoints>
</Item>
</Trans>
<Trans>
9              deny_i        7              +Free+       
<Item>
<Position>
3350.900000    602.780000    
</Position>
<BasePoints>
-126.300000    13.190000      0              0              -66.770322     16.617949     
-76            18.410000      -85            19.870000      -93.700000     20.830000     
-122.500000    24.050000      -130.100000    24.160000      -159           20.830000     
-165.600000    20.060000      -172.500000    18.980000      -193.001821    13.919863     
-249.400000    0             
</BasePoints>
</Item>
</Trans>
<Trans>
9              pick_box_i    12             +Free+       
<Item>
<Position>
3350.900000    602.780000    
</Position>
<BasePoints>
126.300000     111.800000     0              0              42.673468      53.910151     
86             110.550000     153.700000     198.870000     207.640342     268.232804    
245.300000     316.660000    
</BasePoints>
</Item>
</Trans>
<Trans>
10             tin_k         11             +Free+       
<Item>
<Position>
3350.900000    919.440000    
</Position>
<BasePoints>
-1131          139.560000     0              0              -48.951028     13.446548     
-85.600000     23.260000      -137.600000    36             -184           43.060000     
-712.200000    123.560000     -847           151.360000     -1381.300000   151.360000    
-2003.300000   151.360000     -2003.300000   151.360000     -2003.300000   151.360000    
-2072          151.360000     -2150          148.560000     -2221.362699   145.062822    
-2290.800000   141.660000    
</BasePoints>
</Item>
</Trans>
<Trans>
10             tout_k        11             +Free+       
<Item>
<Position>
3350.900000    919.440000    
</Position>
<BasePoints>
-1131          196.560000     0              0              -37.026940     34.571784     
-101           92.860000      -241.900000    204.160000     -384.200000    204.160000    
-2003.300000   204.160000     -2003.300000   204.160000     -2003.300000   204.160000    
-2074.900000   204.160000     -2154.400000   185.260000     -2224.598236   162.821268    
-2290.800000   141.660000    
</BasePoints>
</Item>
</Trans>
<Trans>
10             request_i     12             +Free+       
<Item>
<Position>
3350.900000    919.440000    
</Position>
<BasePoints>
126.300000     -7.630000      0              0              50.749425      0             
85.100000      0              130.800000     0              183.912968     0             
245.300000     0             
</BasePoints>
</Item>
</Trans>
<Trans>
10             goal_reached_i  13             +Free+       
<Item>
<Position>
3350.900000    919.440000    
</Position>
<BasePoints>
-811.500000    15.980000      0              0              -50.104068     7.436080      
-100.100000    14.390000      -179.200000    23.620000      -248           23.620000     
-1261          23.620000      -1261          23.620000      -1261          23.620000     
-1328          23.620000      -1352.600000   37.950000      -1410.500000   4.170000      
-1429.600000   -7             -1445.800000   -24.460000     -1464.476464   -55.642889    
-1495.200000   -106.940000   
</BasePoints>
</Item>
</Trans>
<Trans>
11             broadcast     11             +Free+       
<Item>
<Position>
1060.100000    1061.100000   
</Position>
<BasePoints>
0              -102.120000    0              0              -23.622598     -65.412571    
-22.100000     -81.990000     -14.200000     -94.480000     0              -94.480000    
9.800000       -94.480000     16.600000      -88.580000     12.790393      -68.251387    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
11             goal_reached_i  14             +Free+       
<Item>
<Position>
1060.100000    1061.100000   
</Position>
<BasePoints>
140.400000     -179.850000    0              0              50.648448      -47.745671    
97.900000      -93.690000     169.800000     -163.480000    232.791667     -223.927671   
288.900000     -277.770000   
</BasePoints>
</Item>
</Trans>
<Trans>
11             request_i     20             +Free+       
<Item>
<Position>
1060.100000    1061.100000   
</Position>
<BasePoints>
1031.200000    -65.960000     0              0              69.091919      -7.455962     
77.900000      -8.300000      86.400000      -9.100000      94.500000      -9.700000     
739.100000     -60.600000     902.900000     -32.900000     1547.200000    -87.490000    
1675.200000    -98.340000     1711.100000    -84.480000     1834.400000    -120.820000   
1882.800000    -135.100000    1933.700000    -160.100000    1984.710416    -189.517427   
2041.400000    -222.210000   
</BasePoints>
</Item>
</Trans>
<Trans>
12             deny_i        10             +Free+       
<Item>
<Position>
3596.200000    919.440000    
</Position>
<BasePoints>
-119           13.200000      0              0              -59.133759     16.093114     
-68.500000     18.130000      -77.600000     19.780000      -86.400000     20.840000     
-115.200000    24.320000      -122.800000    24.050000      -151.600000    20.840000     
-161.600000    19.720000      -172.100000    17.950000      -196.047406    12.077651     
-245.300000    0             
</BasePoints>
</Item>
</Trans>
<Trans>
12             goal_reached_i  15             +Free+       
<Item>
<Position>
3596.200000    919.440000    
</Position>
<BasePoints>
-774.500000    -15.970000     0              0              -55.989522     24.950768     
-90            39.090000      -134.900000    55.290000      -176.600000    62.500000     
-236.800000    72.910000      -253.200000    67.620000      -314           62.500000     
-735           27.120000      -1231          -77.530000     -1436.157716   -123.482632   
-1504.900000   -138.880000   
</BasePoints>
</Item>
</Trans>
<Trans>
13             drop_box_i    1              +Free+       
<Item>
<Position>
1855.700000    812.500000    
</Position>
<BasePoints>
-923.390000    92.360000      0              0              -50.051593     32.643210     
-98.400000     61.910000      -175.400000    100            -248.500000    100           
-1573.980000   100            -1573.980000   100            -1573.980000   100           
-1709.740000   100            -1748.720000   -390.200000    -1759.593350   -571.626534   
-1762.043000   -612.500000   
</BasePoints>
</Item>
</Trans>
<Trans>
13             tin_k         14             +Free+       
<Item>
<Position>
1855.700000    812.500000    
</Position>
<BasePoints>
-249.900000    -27.080000     0              0              -59.582433     -3.373344     
-146.900000    -8.420000      -311.800000    -17.960000     -429.007915    -24.701410    
-506.700000    -29.170000    
</BasePoints>
</Item>
</Trans>
<Trans>
13             tout_k        14             +Free+       
<Item>
<Position>
1855.700000    812.500000    
</Position>
<BasePoints>
-249.900000    2.080000       0              0              -59.406679     6.458113      
-124.400000    12.580000      -231.700000    19.430000      -323.500000    9.720000      
-354.400000    6.450000       -387.800000    0.150000       -431.342696    -10.587358    
-506.700000    -29.170000    
</BasePoints>
</Item>
</Trans>
<Trans>
13             request_i     15             +Free+       
<Item>
<Position>
1855.700000    812.500000    
</Position>
<BasePoints>
112.500000     -27.080000     0              0              59.150636      -7.933145     
87.300000      -11.790000     121.100000     -16.420000     165.938578     -22.497683    
235.600000     -31.940000    
</BasePoints>
</Item>
</Trans>
<Trans>
14             drop_box_i    2              +Free+       
<Item>
<Position>
1349           783.330000    
</Position>
<BasePoints>
-551.880000    -46.520000     0              0              -76.000867     -15.748017    
-133.800000    -26.580000     -215.300000    -38.890000     -287.500000    -38.890000    
-844.550000    -38.890000     -844.550000    -38.890000     -844.550000    -38.890000    
-984.390000    -38.890000     -1036.800000   -234.230000    -1056.543728   -358.990813   
-1065.890000   -418.050000   
</BasePoints>
</Item>
</Trans>
<Trans>
14             broadcast     14             +Free+       
<Item>
<Position>
1349           783.330000    
</Position>
<BasePoints>
0              -110.310000    0              0              -23.276712     -74.215753    
-21.200000     -90.710000     -13.400000     -102.670000    0              -102.670000   
9.100000       -102.670000    15.700000      -97.020000     12.403578      -76.649371    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
14             request_i     16             +Free+       
<Item>
<Position>
1349           783.330000    
</Position>
<BasePoints>
130.400000     -75.690000     0              0              71.188919      -31.327111    
103.900000     -45.950000     142.500000     -63.280000     189.492673     -84.087517    
256.800000     -113.890000   
</BasePoints>
</Item>
</Trans>
<Trans>
15             drop_box_i    3              +Free+       
<Item>
<Position>
2091.300000    780.560000    
</Position>
<BasePoints>
128.600000     -478.480000    0              0              24.475514      -66.023947    
56.300000      -157.010000    114.700000     -325.900000    161.900000     -470.840000   
181.600000     -531.290000    203.100000     -600.620000    222.932455     -663.301762   
238.500000     -712.504000   
</BasePoints>
</Item>
</Trans>
<Trans>
15             deny_i        13             +Free+       
<Item>
<Position>
2091.300000    780.560000    
</Position>
<BasePoints>
-123.100000    32.630000      0              0              -63.275147     30.649522     
-73.900000     34.670000      -84.700000     38.100000      -95.300000     40.270000     
-116.900000    44.700000      -140.700000    44.920000      -176.380345    40.039801     
-235.600000    31.940000     
</BasePoints>
</Item>
</Trans>
<Trans>
16             accept_i      13             +Free+       
<Item>
<Position>
1605.800000    669.440000    
</Position>
<BasePoints>
131.900000     29.870000      0              0              69.227526      24.862146     
106.700000     39.100000      148.400000     56.040000      165.200000     66.670000     
176.200000     73.610000      187            82.050000      207.086353     101.532804    
249.900000     143.060000    
</BasePoints>
</Item>
</Trans>
<Trans>
16             deny_i        13             +Free+       
<Item>
<Position>
1605.800000    669.440000    
</Position>
<BasePoints>
131.900000     77.090000      0              0              53.091040      50.849859     
67.200000      63.120000      82.900000      75.230000      98.600000      84.730000     
113.100000     93.530000      147.700000     107.150000     193.510513     123.246434    
249.900000     143.060000    
</BasePoints>
</Item>
</Trans>
<Trans>
16             drop_box_i    17             +Free+       
<Item>
<Position>
1605.800000    669.440000    
</Position>
<BasePoints>
131.900000     -179.860000    0              0              56.987554      -46.498250    
92.400000      -78.710000     136.700000     -124.140000    165.200000     -172.220000   
200.500000     -231.530000    222.700000     -307.500000    237.885216     -376.196353   
249.900000     -430.550000   
</BasePoints>
</Item>
</Trans>
<Trans>
17             accept_i      1              +Free+       
<Item>
<Position>
1855.700000    238.890000    
</Position>
<BasePoints>
-923.390000    -20.140000     0              0              -55.525842     -4.318676     
-105.900000    -7.950000      -182.200000    -12.500000     -248.500000    -12.500000    
-1573.980000   -12.500000     -1573.980000   -12.500000     -1573.980000   -12.500000    
-1619.490000   -12.500000     -1670.630000   -20.290000     -1721.960657   -30.734359    
-1762.043000   -38.890000    
</BasePoints>
</Item>
</Trans>
<Trans>
17             deny_i        1              +Free+       
<Item>
<Position>
1855.700000    238.890000    
</Position>
<BasePoints>
-923.390000    -72.920000     0              0              -51.448529     -21.358632    
-100.700000    -40.410000     -178.400000    -65.280000     -248.500000    -65.280000    
-1573.980000   -65.280000     -1573.980000   -65.280000     -1573.980000   -65.280000    
-1619.490000   -65.280000     -1670.630000   -57.490000     -1721.960657   -47.045641    
-1762.043000   -38.890000    
</BasePoints>
</Item>
</Trans>
<Trans>
17             work_i        18             +Free+       
<Item>
<Position>
1855.700000    238.890000    
</Position>
<BasePoints>
364.200000     43.750000      0              0              54.966614      8.304522      
174.900000     26.900000      462.300000     71.450000      616.590050     95.213734     
683.700000     105.550000    
</BasePoints>
</Item>
</Trans>
<Trans>
18             accept_i      4              +Free+       
<Item>
<Position>
2539.400000    344.440000    
</Position>
<BasePoints>
138.700000     -65.970000     0              0              64.419516      -21.363818    
74.200000      -24.580000     83.800000      -27.690000     92.900000      -30.550000    
133.900000     -43.460000     180.200000     -57.160000     230.352785     -71.380599    
282.300000     -86.110000    
</BasePoints>
</Item>
</Trans>
<Trans>
18             deny_i        4              +Free+       
<Item>
<Position>
2539.400000    344.440000    
</Position>
<BasePoints>
138.700000     -17.360000     0              0              67.534192      6.036242      
103.500000     6.970000       147.600000     4.250000       184.500000     -9.720000     
202            -16.340000     218.800000     -27.380000     242.674625     -49.461208    
282.300000     -86.110000    
</BasePoints>
</Item>
</Trans>
<Trans>
18             goal_reached_i  19             +Free+       
<Item>
<Position>
2539.400000    344.440000    
</Position>
<BasePoints>
138.700000     86.810000      0              0              49.447299      46.604580     
96.200000      91.890000      167.500000     160.990000     229.720853     220.527986    
282.300000     270.840000    
</BasePoints>
</Item>
</Trans>
<Trans>
19             accept_i      7              +Free+       
<Item>
<Position>
2821.700000    615.280000    
</Position>
<BasePoints>
143.600000     -17.360000     0              0              72.614198      -4.263220     
81.300000      -4.740000      89.700000      -5.170000      97.800000      -5.560000     
133.800000     -7.290000      173.900000     -8.870000      221.782685     -10.511305    
279.800000     -12.500000    
</BasePoints>
</Item>
</Trans>
<Trans>
19             deny_i        7              +Free+       
<Item>
<Position>
2821.700000    615.280000    
</Position>
<BasePoints>
143.600000     4.860000       0              0              71.986552      11.436091     
107.500000     15.480000      150.800000     17.810000      189.400000     12.500000     
196.400000     11.540000      203.600000     10.170000      224.049842     4.086038      
279.800000     -12.500000    
</BasePoints>
</Item>
</Trans>
<Trans>
19             pick_box_i    20             +Free+       
<Item>
<Position>
2821.700000    615.280000    
</Position>
<BasePoints>
143.600000     70.140000      0              0              57.230307      45.116226     
103.700000     82.580000      169.200000     135.480000     228.562634     182.782251    
279.800000     223.610000    
</BasePoints>
</Item>
</Trans>
<Trans>
20             accept_i      10             +Free+       
<Item>
<Position>
3101.500000    838.890000    
</Position>
<BasePoints>
123.100000     9.030000       0              0              64.346738      11.484798     
92.900000      17.510000      126.400000     25.760000      155.700000     36.110000     
167.500000     40.280000      179.800000     45.530000      204.064973     57.739186     
249.400000     80.550000     
</BasePoints>
</Item>
</Trans>
<Trans>
20             deny_i        10             +Free+       
<Item>
<Position>
3101.500000    838.890000    
</Position>
<BasePoints>
123.100000     49.300000      0              0              53.150843      38.226566     
64.900000      45.400000      77.700000      52.170000      90.400000      56.940000     
120.300000     68.130000      155.100000     74.030000      198.842820     77.054424     
249.400000     80.550000     
</BasePoints>
</Item>
</Trans>
<Trans>
20             goal_reached_i  16             +Free+       
<Item>
<Position>
3101.500000    838.890000    
</Position>
<BasePoints>
-771.700000    -136.810000    0              0              -64.293079     -11.727534    
-215.800000    -39.450000     -608.700000    -108.460000    -939.900000    -141.670000   
-1104.900000   -158.210000    -1298.500000   -165.060000    -1422.086987   -167.811252   
-1495.700000   -169.450000   
</BasePoints>
</Item>
</Trans>
<Trans>
16             deny_i        14             +Line+       
<Item>
<Position>
1605.800000    669.440000    
</Position>
<BasePoints>
-132.454150    47.803673      0              0              -67.334573     29.862673     
-106.396453    47.186496      -145.458141    64.510232      -185.703719    82.359021     
-256.800000    113.890000    
</BasePoints>
</Item>
</Trans>
<Trans>
17             deny_i        2              +Line+       
<Item>
<Position>
1855.700000    238.890000    
</Position>
<BasePoints>
-787.096123    53.227142      0              0              -55.506135     4.461061      
-536.999556    43.158976      -1017.944879   81.812839      -1513.059897   121.605530    
-1572.590000   126.390000    
</BasePoints>
</Item>
</Trans>
<Trans>
18             deny_i        5              +Line+       
<Item>
<Position>
2539.400000    344.440000    
</Position>
<BasePoints>
-1017.209882   33.759244      0              0              -67.843571     2.919173      
-693.058696    29.820923      -1317.603895   56.693848      -1960.862348   84.371966     
-2033.560000   87.500000     
</BasePoints>
</Item>
</Trans>
<Trans>
19             deny_i        8              +Line+       
<Item>
<Position>
2821.700000    615.280000    
</Position>
<BasePoints>
-1012.248807   -14.169915     0              0              -72.727689     -0.299592     
-691.342665    -2.847898      -1310.520124   -5.398521      -1947.642268   -8.023065     
-2024.580000   -8.340000     
</BasePoints>
</Item>
</Trans>
<Trans>
20             deny_i        11             +Line+       
<Item>
<Position>
3101.500000    838.890000    
</Position>
<BasePoints>
-1021.782126   101.163722     0              0              -64.980343     7.073225      
-694.375033    75.583950      -1323.959539   144.115337     -1972.354422   214.694267    
-2041.400000   222.210000    
</BasePoints>
</Item>
</Trans>
</TransRel>
</GraphData>
</VioModels>
<VioLayout>
0             =AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAB7FA===  0              150            220            0.073508       0             
220           
</VioLayout>
</VioSystem>
</Value>
</Variable>
<Variable>
E_TaskTransmitter_Robot_i  Alphabet      +Visual+     
<Value>
<VioAlphabet>
<E_TaskTransmitter_Robot_i>
tin_k         +C+           broadcast     request_i     +C+           accept_i     
+C+           tout_k        +C+           deny_i        +C+           work_i       
+C+           goal_reached_i  pick_box_i    drop_box_i   
</E_TaskTransmitter_Robot_i>
<VioData>
=AAAACgAAAAoAdABpAG4AXwBrAAAAEgBiAHIAbwBhAGQAYwBhAHMAdAAAABIAcgBlAHEAdQBlAHMAdABfAGkAAAAQAGEAYwBjAGUAcAB0AF8AaQAAAAwAdABvAHUAdABfAGsAAAAMAGQAZQBuAHkAXwBpAAAADAB3AG8AcgBrAF8AaQAAABwAZwBvAGEAbABfAHIAZQBhAGMAaABlAGQAXwBpAAAAFABwAGkAYwBrAF8AYgBvAHgAXwBpAAAAFABkAHIAbwBwAF8AYgBvAHgAXwBp= </VioData>
<VioLayout>
0              150            150           
</VioLayout>
</VioAlphabet>
</Value>
</Variable>
<Variable>
Hsinv_Request_i_TaskTransmitter_Robot_i  System        +Visual+     
<Value>
<VioSystem>
<Generator name="Hsinv_Request_i_TaskTransmitter_Robot_i" ftype="System">

<Alphabet>
<Event name="tin_k">
<Controllable/>
</Event>
<Event name="broadcast"/>
<Event name="request_i">
<Controllable/>
</Event>
<Event name="accept_i">
<Controllable/>
</Event>
<Event name="tout_k">
<Controllable/>
</Event>
<Event name="deny_i">
<Controllable/>
</Event>
<Event name="work_i">
<Controllable/>
</Event>
<Event name="goal_reached_i"/>
<Event name="pick_box_i"/>
<Event name="drop_box_i"/>
</Alphabet>

<StateSet>
<State id="1" name="Idle">
<Initial/>
<Marked/>
</State>
<State id="2" name="Request"/>
<State id="3" name="Wait"/>
<State id="4" name="Work"/>
<State id="5" name="WorkDone"/>
</StateSet>

<TransitionRelation>
<Transition x1="1" event="tin_k" x2="1"/>
<Transition x1="1" event="broadcast" x2="2"/>
<Transition x1="1" event="tout_k" x2="1"/>
<Transition x1="1" event="goal_reached_i" x2="1"/>
<Transition x1="1" event="pick_box_i" x2="1"/>
<Transition x1="2" event="tin_k" x2="2"/>
<Transition x1="2" event="broadcast" x2="2"/>
<Transition x1="2" event="request_i" x2="3"/>
<Transition x1="2" event="tout_k" x2="2"/>
<Transition x1="2" event="goal_reached_i" x2="2"/>
<Transition x1="2" event="pick_box_i" x2="2"/>
<Transition x1="3" event="tin_k" x2="3"/>
<Transition x1="3" event="accept_i" x2="4"/>
<Transition x1="3" event="tout_k" x2="3"/>
<Transition x1="3" event="deny_i" x2="1"/>
<Transition x1="3" event="goal_reached_i" x2="3"/>
<Transition x1="3" event="pick_box_i" x2="3"/>
<Transition x1="4" event="tin_k" x2="4"/>
<Transition x1="4" event="broadcast" x2="4"/>
<Transition x1="4" event="tout_k" x2="4"/>
<Transition x1="4" event="work_i" x2="5"/>
<Transition x1="4" event="goal_reached_i" x2="4"/>
<Transition x1="4" event="pick_box_i" x2="4"/>
<Transition x1="5" event="tin_k" x2="5"/>
<Transition x1="5" event="broadcast" x2="5"/>
<Transition x1="5" event="tout_k" x2="5"/>
<Transition x1="5" event="goal_reached_i" x2="5"/>
<Transition x1="5" event="pick_box_i" x2="5"/>
<Transition x1="5" event="drop_box_i" x2="1"/>
</TransitionRelation>

</Generator>
<VioModels>
<TransitionList>
=AAAAHQEAAAABAAAACgB0AGkAbgBfAGsAAAABAQAAAAEAAAASAGIAcgBvAGEAZABjAGEAcwB0AAAAAgEAAAABAAAADAB0AG8AdQB0AF8AawAAAAEBAAAAAQAAABwAZwBvAGEAbABfAHIAZQBhAGMAaABlAGQAXwBpAAAAAQEAAAABAAAAFABwAGkAYwBrAF8AYgBvAHgAXwBpAAAAAQEAAAACAAAACgB0AGkAbgBfAGsAAAACAQAAAAIAAAASAHIAZQBxAHUAZQBzAHQAXwBpAAAAAwEAAAACAAAADAB0AG8AdQB0AF8AawAAAAIBAAAAAgAAABwAZwBvAGEAbABfAHIAZQBhAGMAaABlAGQAXwBpAAAAAgEAAAACAAAAFABwAGkAYwBrAF8AYgBvAHgAXwBpAAAAAgEAAAADAAAACgB0AGkAbgBfAGsAAAADAQAAAAMAAAAQAGEAYwBjAGUAcAB0AF8AaQAAAAQBAAAAAwAAAAwAdABvAHUAdABfAGsAAAADAQAAAAMAAAAMAGQAZQBuAHkAXwBpAAAAAQEAAAADAAAAHABnAG8AYQBsAF8AcgBlAGEAYwBoAGUAZABfAGkAAAADAQAAAAMAAAAUAHAAaQBjAGsAXwBiAG8AeABfAGkAAAADAQAAAAQAAAAKAHQAaQBuAF8AawAAAAQBAAAABAAAAAwAdABvAHUAdABfAGsAAAAEAQAAAAQAAAAcAGcAbwBhAGwAXwByAGUAYQBjAGgAZQBkAF8AaQAAAAQBAAAABAAAABQAcABpAGMAawBfAGIAbwB4AF8AaQAAAAQBAAAAAgAAABIAYgByAG8AYQBkAGMAYQBzAHQAAAACAQAAAAQAAAAMAHcAbwByAGsAXwBpAAAABQEAAAAFAAAACgB0AGkAbgBfAGsAAAAFAQAAAAUAAAAMAHQAbwB1AHQAXwBrAAAABQEAAAAFAAAAHABnAG8AYQBsAF8AcgBlAGEAYwBoAGUAZABfAGkAAAAFAQAAAAUAAAAUAHAAaQBjAGsAXwBiAG8AeABfAGkAAAAFAQAAAAUAAAAUAGQAcgBvAHAAXwBiAG8AeABfAGkAAAABAQAAAAQAAAASAGIAcgBvAGEAZABjAGEAcwB0AAAABAEAAAAFAAAAEgBiAHIAbwBhAGQAYwBhAHMAdAAAAAU== </TransitionList>
<StateList>
=AAAABQIAAAABAgAAAAICAAAAAwIAAAAEAgAAAAU== </StateList>
<EventList>
=AAAACgMAAAAKAHQAaQBuAF8AawMAAAASAGIAcgBvAGEAZABjAGEAcwB0AwAAABIAcgBlAHEAdQBlAHMAdABfAGkDAAAAEABhAGMAYwBlAHAAdABfAGkDAAAADAB0AG8AdQB0AF8AawMAAAAMAGQAZQBuAHkAXwBpAwAAAAwAdwBvAHIAawBfAGkDAAAAHABnAG8AYQBsAF8AcgBlAGEAYwBoAGUAZABfAGkDAAAAFABwAGkAYwBrAF8AYgBvAHgAXwBpAwAAABQAZAByAG8AcABfAGIAbwB4AF8AaQ=== </EventList>
<GraphData>
<States>
<State>
1             
<Item>
<Position>
78.941000      328.050000    
</Position>
<BasePoints>
0              0              26.163000      26.163000      -78.246560     0             
-26.187084     0             
</BasePoints>
</Item>
</State>
<State>
2             
<Item>
<Position>
258.160000     244.720000    
</Position>
<BasePoints>
0              0              43.332000      43.332000      -60            0             
-43.374699     0             
</BasePoints>
</Item>
</State>
<State>
3             
<Item>
<Position>
436.480000     323.890000    
</Position>
<BasePoints>
0              0              29.433000      29.433000      -60            0             
-29.414018     0             
</BasePoints>
</Item>
</State>
<State>
4             
<Item>
<Position>
599.190000     369.720000    
</Position>
<BasePoints>
0              0              31.886000      31.886000      -60            0             
-31.901770     0             
</BasePoints>
</Item>
</State>
<State>
5             
<Item>
<Position>
0              475.606000    
</Position>
<BasePoints>
0              0              30             30             -60            0             
-29.999400     0             
</BasePoints>
</Item>
</State>
</States>
<TransRel>
<Trans>
1              tin_k         1              +Free+       
<Item>
<Position>
78.941000      328.050000    
</Position>
<BasePoints>
0              -58.800000     0              0              -2.002237      -26.062749    
-2.302000      -39.450000     -1.628000      -51.160000     0              -51.160000    
0.991000       -51.160000     1.629000       -46.810000     0.911020       -26.178531    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
1              tout_k        1              +Free+       
<Item>
<Position>
78.941000      328.050000    
</Position>
<BasePoints>
0              -83.800000     0              0              -3.371418      -25.935510    
-5.104000      -50.210000     -3.972000      -76.160000     0              -76.160000    
3.195000       -76.160000     4.553000       -59.360000     2.001350       -26.092719    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
1              goal_reached_i  1              +Free+       
<Item>
<Position>
78.941000      328.050000    
</Position>
<BasePoints>
0              -108.800000    0              0              -4.256247      -25.808420    
-7.935000      -60            -6.511000      -101.160000    0              -101.160000   
5.645000       -101.160000    7.466000       -70.200000     2.771391       -26.058346    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
1              pick_box_i    1              +Free+       
<Item>
<Position>
78.941000      328.050000    
</Position>
<BasePoints>
0              -133.800000    0              0              -4.856382      -25.705028    
-10.753000     -69.560000     -9.115000      -126.160000    0              -126.160000   
8.153000       -126.160000    10.323000      -80.870000     3.316658       -25.982572    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
1              drop_box_i    1              +Free+       
<Item>
<Position>
78.941000      328.050000    
</Position>
<BasePoints>
0              -158.800000    0              0              -5.329098      -25.654541    
-13.575000     -78.420000     -11.801000     -151.160000    0              -151.160000   
10.786000      -151.160000    13.195000      -90.380000     3.786584       -25.936447    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
1              broadcast     2              +Free+       
<Item>
<Position>
78.941000      328.050000    
</Position>
<BasePoints>
81.019000      -59.020000     0              0              23.900554      -10.636139    
49.889000      -22.900000     92.819000      -43.180000     139.869681     -65.044408    
179.219000     -83.330000    
</BasePoints>
</Item>
</Trans>
<Trans>
2              tin_k         2              +Free+       
<Item>
<Position>
258.160000     244.720000    
</Position>
<BasePoints>
0              -75.970000     0              0              -3.529534      -43.241785    
-3.510000      -57.300000     -2.330000      -68.330000     0              -68.330000    
1.450000       -68.330000     2.460000       -64.020000     1.664819       -43.325911    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
2              tout_k        2              +Free+       
<Item>
<Position>
258.160000     244.720000    
</Position>
<BasePoints>
0              -100.970000    0              0              -6.594383      -42.903210    
-8.120000      -69.030000     -5.910000      -93.330000     0              -93.330000    
4.790000       -93.330000     7.150000       -77.290000     3.998643       -43.224497    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
2              goal_reached_i  2              +Free+       
<Item>
<Position>
258.160000     244.720000    
</Position>
<BasePoints>
0              -125.970000    0              0              -8.878493      -42.500655    
-12.920000     -79.330000     -9.960000      -118.330000    0              -118.330000   
8.710000       -118.330000    12.070000      -88.470000     5.858177       -42.938930    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
2              pick_box_i    2              +Free+       
<Item>
<Position>
258.160000     244.720000    
</Position>
<BasePoints>
0              -150.970000    0              0              -10.528561     -42.014634    
-17.730000     -89.300000     -14.210000     -143.330000    0              -143.330000   
12.810000      -143.330000    16.930000      -99.340000     7.291122       -42.782046    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
2              drop_box_i    2              +Free+       
<Item>
<Position>
258.160000     244.720000    
</Position>
<BasePoints>
0              -175.970000    0              0              -11.823464     -41.675967    
-22.540000     -98.810000     -18.590000     -168.331000    0              -168.331000   
17.060000      -168.331000    21.790000      -109.740000    8.447857       -42.545562    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
2              request_i     3              +Free+       
<Item>
<Position>
258.160000     244.720000    
</Position>
<BasePoints>
96.110000      25.700000      0              0              39.792014      17.318649     
69.010000      30.500000      108.790000     48.440000      151.363622     67.256157     
178.320000     79.170000     
</BasePoints>
</Item>
</Trans>
<Trans>
3              deny_i        1              +Free+       
<Item>
<Position>
436.480000     323.890000    
</Position>
<BasePoints>
-178.320000    -4.860000      0              0              -29.419614     0.326775      
-92.820000     1.070000       -245.060000    2.860000       -331.337976    3.857176      
-357.539000    4.160000      
</BasePoints>
</Item>
</Trans>
<Trans>
3              tin_k         3              +Free+       
<Item>
<Position>
436.480000     323.890000    
</Position>
<BasePoints>
0              -62.070000     0              0              -2.830749      -29.274084    
-3.150000      -42.950000     -2.190000      -54.440000     0              -54.440000    
1.330000       -54.440000     2.210000       -50.170000     1.297076       -29.445394    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
3              tout_k        3              +Free+       
<Item>
<Position>
436.480000     323.890000    
</Position>
<BasePoints>
0              -87.070000     0              0              -4.896395      -29.071103    
-7.060000      -53.830000     -5.410000      -79.440000     0              -79.440000    
4.350000       -79.440000     6.270000       -62.860000     2.920685       -29.281381    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
3              goal_reached_i  3              +Free+       
<Item>
<Position>
436.480000     323.890000    
</Position>
<BasePoints>
0              -112.070000    0              0              -6.255379      -28.756815    
-11.030000     -63.620000     -8.930000      -104.440000    0              -104.440000   
7.740000       -104.440000    10.350000      -73.740000     4.093494       -29.164663    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
3              pick_box_i    3              +Free+       
<Item>
<Position>
436.480000     323.890000    
</Position>
<BasePoints>
0              -137.070000    0              0              -7.212784      -28.581407    
-14.990000     -73.020000     -12.580000     -129.440000    0              -129.440000   
11.300000      -129.440000    14.390000      -83.900000     4.978953       -29.029478    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
3              drop_box_i    3              +Free+       
<Item>
<Position>
436.480000     323.890000    
</Position>
<BasePoints>
0              -162.070000    0              0              -7.912085      -28.411577    
-18.930000     -82.150000     -16.290000     -154.440000    0              -154.440000   
14.890000      -154.440000    18.380000      -94.040000     5.653988       -28.928238    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
3              accept_i      4              +Free+       
<Item>
<Position>
436.480000     323.890000    
</Position>
<BasePoints>
80.130000      9.030000       0              0              28.435686      7.747029      
53.070000      14.810000      89.640000      25.290000      131.955039     37.184771     
162.710000     45.830000     
</BasePoints>
</Item>
</Trans>
<Trans>
4              work_i        1              +Free+       
<Item>
<Position>
599.190000     369.720000    
</Position>
<BasePoints>
-244.920000    -17.360000     0              0              -31.855473     0.196095      
-70.280000     0.240000       -135.960000    -0.340000      -192.140000    -4.170000     
-296.600000    -11.290000     -419.050000    -27.450000     -494.308348    -38.024944    
-520.249000    -41.670000    
</BasePoints>
</Item>
</Trans>
<Trans>
4              tin_k         4              +Free+       
<Item>
<Position>
599.190000     369.720000    
</Position>
<BasePoints>
0              -64.520000     0              0              -2.958970      -31.720167    
-3.200000      -45.540000     -2.210000      -56.880000     0              -56.880000    
1.340000       -56.880000     2.230000       -52.670000     1.348014       -31.838512    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
4              tout_k        4              +Free+       
<Item>
<Position>
599.190000     369.720000    
</Position>
<BasePoints>
0              -89.520000     0              0              -5.219701      -31.447948    
-7.250000      -56.370000     -5.510000      -81.880000     0              -81.880000    
4.470000       -81.880000     6.460000       -65.040000     3.157211       -31.787151    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
4              goal_reached_i  4              +Free+       
<Item>
<Position>
599.190000     369.720000    
</Position>
<BasePoints>
0              -114.520000    0              0              -6.736444      -31.209855    
-11.360000     -66.380000     -9.110000      -106.880000    0              -106.880000   
7.900000       -106.880000    10.630000      -76.430000     4.391091       -31.572068    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
4              pick_box_i    4              +Free+       
<Item>
<Position>
599.190000     369.720000    
</Position>
<BasePoints>
0              -139.520000    0              0              -7.791529      -30.967100    
-15.450000     -76.050000     -12.840000     -131.880000    0              -131.880000   
11.530000      -131.880000    14.810000      -86.820000     5.365717       -31.455203    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
4              drop_box_i    4              +Free+       
<Item>
<Position>
599.190000     369.720000    
</Position>
<BasePoints>
0              -164.520000    0              0              -8.591406      -30.769217    
-19.550000     -85.160000     -16.680000     -156.880000    0              -156.880000   
15.310000      -156.880000    18.980000      -96.440000     6.160123       -31.300433    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
2              broadcast     2              +Spline+     
<Item>
<Position>
258.160000     244.720000    
</Position>
<BasePoints>
0              -116.279297    0              0              19.417401      -38.834802    
58.139648      -116.279297    -58.139648     -116.279297    -19.417678     -38.835357    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
4              work_i        5              +Line+       
<Item>
<Position>
599.190000     369.720000    
</Position>
<BasePoints>
-301.335190    43.095577      0              0              -31.396602     5.548258      
-208.955810    36.925674      -386.606282    68.319219      -569.619179    100.660385    
-599.190000    105.886000    
</BasePoints>
</Item>
</Trans>
<Trans>
5              tin_k         5              +Spline+     
<Item>
<Position>
0              475.606000    
</Position>
<BasePoints>
0              -80.595703     0              0              13.432387      -26.864773    
40.297852      -80.595703     -40.297852     -80.595703     -13.419461     -26.838922    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
5              tout_k        5              +Spline+     
<Item>
<Position>
0              475.606000    
</Position>
<BasePoints>
0              -80.595703     0              0              13.432387      -26.864773    
40.297852      -80.595703     -40.297852     -80.595703     -13.419461     -26.838922    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
5              goal_reached_i  5              +Spline+     
<Item>
<Position>
0              475.606000    
</Position>
<BasePoints>
0              -80.595703     0              0              13.432387      -26.864773    
40.297852      -80.595703     -40.297852     -80.595703     -13.419461     -26.838922    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
5              pick_box_i    5              +Spline+     
<Item>
<Position>
0              475.606000    
</Position>
<BasePoints>
0              -80.595703     0              0              13.432387      -26.864773    
40.297852      -80.595703     -40.297852     -80.595703     -13.419461     -26.838922    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
5              drop_box_i    1              +Line+       
<Item>
<Position>
0              475.606000    
</Position>
<BasePoints>
48.287956      -69.060748     0              0              14.145924      -26.441466    
31.445346      -58.777434     48.744526      -91.112948     66.585252      -124.460716   
78.941000      -147.556000   
</BasePoints>
</Item>
</Trans>
<Trans>
4              broadcast     4              +Spline+     
<Item>
<Position>
599.190000     369.720000    
</Position>
<BasePoints>
0              -85.517578     0              0              14.266618      -28.533235    
42.758789      -85.517578     -42.758789     -85.517578     -14.280727     -28.561453    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
5              broadcast     5              +Spline+     
<Item>
<Position>
0              475.606000    
</Position>
<BasePoints>
0              -80.595703     0              0              13.419308      -26.838616    
40.297852      -80.595703     -40.297852     -80.595703     -13.419461     -26.838922    
0              0             
</BasePoints>
</Item>
</Trans>
</TransRel>
</GraphData>
</VioModels>
<VioLayout>
0             =AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAACKQQ===  0              150            499            1.188686       0             
500           
</VioLayout>
</VioSystem>
</Value>
</Variable>
<Variable>
Rl_Request_i_TaskTransmitter_Robot_i  System        +Visual+     
<Value>
<VioSystem>
<Generator name="Rl_Request_i_TaskTransmitter_Robot_i" ftype="System">

<Alphabet>
<Event name="tin_k">
<Controllable/>
</Event>
<Event name="broadcast"/>
<Event name="request_i">
<Controllable/>
</Event>
<Event name="accept_i">
<Controllable/>
</Event>
<Event name="tout_k">
<Controllable/>
</Event>
<Event name="deny_i">
<Controllable/>
</Event>
<Event name="work_i">
<Controllable/>
</Event>
<Event name="goal_reached_i"/>
<Event name="pick_box_i"/>
<Event name="drop_box_i"/>
</Alphabet>

<StateSet>
<State id="1" name="Idle|Idle|Idle">
<Initial/>
<Marked/>
</State>
<State id="2" name="Broadcast|Idle|Idle"/>
<State id="3" name="Broadcast|Idle|Request"/>
<State id="4" name="Answer|Idle|Wait"/>
<State id="5" name="Idle|Idle|Work"/>
<State id="6" name="Broadcast|Idle|Work"/>
<State id="7" name="Idle|ToStart|WorkDone"/>
<State id="8" name="Broadcast|ToStart|WorkDone"/>
<State id="9" name="Idle|PickBox|WorkDone"/>
<State id="10" name="Broadcast|PickBox|WorkDone"/>
<State id="11" name="Idle|ToEnd|WorkDone"/>
<State id="12" name="Broadcast|ToEnd|WorkDone"/>
<State id="13" name="Idle|DropBox|WorkDone"/>
<State id="14" name="Broadcast|DropBox|WorkDone"/>
</StateSet>

<TransitionRelation>
<Transition x1="1" event="tin_k" x2="2"/>
<Transition x1="1" event="tout_k" x2="2"/>
<Transition x1="2" event="broadcast" x2="3"/>
<Transition x1="3" event="broadcast" x2="3"/>
<Transition x1="3" event="request_i" x2="4"/>
<Transition x1="4" event="accept_i" x2="5"/>
<Transition x1="4" event="deny_i" x2="2"/>
<Transition x1="5" event="tin_k" x2="6"/>
<Transition x1="5" event="tout_k" x2="6"/>
<Transition x1="5" event="work_i" x2="7"/>
<Transition x1="6" event="broadcast" x2="6"/>
<Transition x1="6" event="work_i" x2="8"/>
<Transition x1="7" event="tin_k" x2="8"/>
<Transition x1="7" event="tout_k" x2="8"/>
<Transition x1="7" event="goal_reached_i" x2="9"/>
<Transition x1="8" event="broadcast" x2="8"/>
<Transition x1="8" event="goal_reached_i" x2="10"/>
<Transition x1="9" event="tin_k" x2="10"/>
<Transition x1="9" event="tout_k" x2="10"/>
<Transition x1="9" event="pick_box_i" x2="11"/>
<Transition x1="10" event="broadcast" x2="10"/>
<Transition x1="10" event="pick_box_i" x2="12"/>
<Transition x1="11" event="tin_k" x2="12"/>
<Transition x1="11" event="tout_k" x2="12"/>
<Transition x1="11" event="goal_reached_i" x2="13"/>
<Transition x1="12" event="broadcast" x2="12"/>
<Transition x1="12" event="goal_reached_i" x2="14"/>
<Transition x1="13" event="tin_k" x2="14"/>
<Transition x1="13" event="tout_k" x2="14"/>
<Transition x1="13" event="drop_box_i" x2="1"/>
<Transition x1="14" event="broadcast" x2="14"/>
<Transition x1="14" event="drop_box_i" x2="2"/>
</TransitionRelation>

</Generator>
<VioModels>
<TransitionList>
=AAAAIAEAAAABAAAACgB0AGkAbgBfAGsAAAACAQAAAAEAAAAMAHQAbwB1AHQAXwBrAAAAAgEAAAACAAAAEgBiAHIAbwBhAGQAYwBhAHMAdAAAAAMBAAAAAwAAABIAcgBlAHEAdQBlAHMAdABfAGkAAAAEAQAAAAQAAAAQAGEAYwBjAGUAcAB0AF8AaQAAAAUBAAAABQAAAAwAdwBvAHIAawBfAGkAAAAHAQAAAAcAAAAcAGcAbwBhAGwAXwByAGUAYQBjAGgAZQBkAF8AaQAAAAkBAAAACQAAABQAcABpAGMAawBfAGIAbwB4AF8AaQAAAAsBAAAACwAAABwAZwBvAGEAbABfAHIAZQBhAGMAaABlAGQAXwBpAAAADQEAAAANAAAAFABkAHIAbwBwAF8AYgBvAHgAXwBpAAAAAQEAAAADAAAAEgBiAHIAbwBhAGQAYwBhAHMAdAAAAAMBAAAABQAAAAoAdABpAG4AXwBrAAAABgEAAAAFAAAADAB0AG8AdQB0AF8AawAAAAYBAAAABgAAABIAYgByAG8AYQBkAGMAYQBzAHQAAAAGAQAAAAYAAAAMAHcAbwByAGsAXwBpAAAACAEAAAAHAAAACgB0AGkAbgBfAGsAAAAIAQAAAAcAAAAMAHQAbwB1AHQAXwBrAAAACAEAAAAIAAAAEgBiAHIAbwBhAGQAYwBhAHMAdAAAAAgBAAAACAAAABwAZwBvAGEAbABfAHIAZQBhAGMAaABlAGQAXwBpAAAACgEAAAAJAAAACgB0AGkAbgBfAGsAAAAKAQAAAAkAAAAMAHQAbwB1AHQAXwBrAAAACgEAAAAKAAAAEgBiAHIAbwBhAGQAYwBhAHMAdAAAAAoBAAAACgAAABQAcABpAGMAawBfAGIAbwB4AF8AaQAAAAwBAAAACwAAAAoAdABpAG4AXwBrAAAADAEAAAALAAAADAB0AG8AdQB0AF8AawAAAAwBAAAADAAAABIAYgByAG8AYQBkAGMAYQBzAHQAAAAMAQAAAAwAAAAcAGcAbwBhAGwAXwByAGUAYQBjAGgAZQBkAF8AaQAAAA4BAAAADQAAAAoAdABpAG4AXwBrAAAADgEAAAANAAAADAB0AG8AdQB0AF8AawAAAA4BAAAADgAAABIAYgByAG8AYQBkAGMAYQBzAHQAAAAOAQAAAA4AAAAUAGQAcgBvAHAAXwBiAG8AeABfAGkAAAACAQAAAAQAAAAMAGQAZQBuAHkAXwBpAAAAAg=== </TransitionList>
<StateList>
=AAAADgIAAAABAgAAAAICAAAAAwIAAAAEAgAAAAUCAAAABwIAAAAJAgAAAAsCAAAADQIAAAAGAgAAAAgCAAAACgIAAAAMAgAAAA4== </StateList>
<EventList>
=AAAACgMAAAAKAHQAaQBuAF8AawMAAAASAGIAcgBvAGEAZABjAGEAcwB0AwAAABIAcgBlAHEAdQBlAHMAdABfAGkDAAAAEABhAGMAYwBlAHAAdABfAGkDAAAADAB0AG8AdQB0AF8AawMAAAAMAGQAZQBuAHkAXwBpAwAAAAwAdwBvAHIAawBfAGkDAAAAHABnAG8AYQBsAF8AcgBlAGEAYwBoAGUAZABfAGkDAAAAFABwAGkAYwBrAF8AYgBvAHgAXwBpAwAAABQAZAByAG8AcABfAGIAbwB4AF8AaQ=== </EventList>
<GraphData>
<States>
<State>
1             
<Item>
<Position>
115.730000     350           
</Position>
<BasePoints>
0              0              62.955000      62.955000      -115.035560    0             
-62.991022     0             
</BasePoints>
</Item>
</State>
<State>
2             
<Item>
<Position>
359.220000     302.780000    
</Position>
<BasePoints>
0              0              87.480000      87.480000      -60            0             
-87.453575     0             
</BasePoints>
</Item>
</State>
<State>
3             
<Item>
<Position>
670.230000     448.610000    
</Position>
<BasePoints>
0              0              105.470000     105.470000     -60            0             
-105.482826    0             
</BasePoints>
</Item>
</State>
<State>
4             
<Item>
<Position>
969.710000     433.330000    
</Position>
<BasePoints>
0              0              80.125000      80.125000      -60            0             
-80.195045     0             
</BasePoints>
</Item>
</State>
<State>
5             
<Item>
<Position>
1224.600000    433.330000    
</Position>
<BasePoints>
0              0              67.860000      67.860000      -60            0             
-67.785301     0             
</BasePoints>
</Item>
</State>
<State>
6             
<Item>
<Position>
1490.500000    480.560000    
</Position>
<BasePoints>
0              0              93.205000      93.205000      -60            0             
-93.307228     0             
</BasePoints>
</Item>
</State>
<State>
7             
<Item>
<Position>
1490.500000    219.440000    
</Position>
<BasePoints>
0              0              102.200000     102.200000     -60            0             
-102.204781    0             
</BasePoints>
</Item>
</State>
<State>
8             
<Item>
<Position>
1872.200000    461.110000    
</Position>
<BasePoints>
0              0              126.725000     126.725000     -60            0             
-126.673050    0             
</BasePoints>
</Item>
</State>
<State>
9             
<Item>
<Position>
1872.200000    162.500000    
</Position>
<BasePoints>
0              0              106.285000     106.285000     -60            0             
-106.185265    0             
</BasePoints>
</Item>
</State>
<State>
10            
<Item>
<Position>
2283.400000    456.940000    
</Position>
<BasePoints>
0              0              131.630000     131.630000     -60            0             
-131.824265    0             
</BasePoints>
</Item>
</State>
<State>
11            
<Item>
<Position>
2283.400000    162.500000    
</Position>
<BasePoints>
0              0              97.295000      97.295000      -60            0             
-97.287712     0             
</BasePoints>
</Item>
</State>
<State>
12            
<Item>
<Position>
2690.400000    216.670000    
</Position>
<BasePoints>
0              0              122.640000     122.640000     -60            0             
-122.458420    0             
</BasePoints>
</Item>
</State>
<State>
13            
<Item>
<Position>
2690.400000    473.610000    
</Position>
<BasePoints>
0              0              108.740000     108.740000     -60            0             
-108.760872    0             
</BasePoints>
</Item>
</State>
<State>
14            
<Item>
<Position>
3099.900000    243.060000    
</Position>
<BasePoints>
0              0              134.085000     134.085000     -60            0             
-134.165726    0             
</BasePoints>
</Item>
</State>
</States>
<TransRel>
<Trans>
1              tin_k         2              +Free+       
<Item>
<Position>
115.730000     350           
</Position>
<BasePoints>
109.480000     -9.030000      0              0              62.910465      4.069975      
84.850000      4.270000       109.180000     3.060000       131.010000     -1.390000     
137.450000     -2.700000      143.980000     -4.380000      163.312410     -11.962894    
243.490000     -47.220000    
</BasePoints>
</Item>
</Trans>
<Trans>
1              tout_k        2              +Free+       
<Item>
<Position>
115.730000     350           
</Position>
<BasePoints>
109.480000     -34.030000     0              0              61.947386      -11.884964    
86.800000      -16.750000     115.930000     -22.470000     157.482500     -30.586150    
243.490000     -47.220000    
</BasePoints>
</Item>
</Trans>
<Trans>
2              broadcast     3              +Free+       
<Item>
<Position>
359.220000     302.780000    
</Position>
<BasePoints>
146.510000     47.910000      0              0              79.491840      36.953665     
116.750000     54.580000      161.780000     75.880000      215.496823     101.222101    
311.010000     145.830000    
</BasePoints>
</Item>
</Trans>
<Trans>
3              broadcast     3              +Free+       
<Item>
<Position>
670.230000     448.610000    
</Position>
<BasePoints>
0              -138.110000    0              0              -28.107368     -101.849171   
-24.150000     -118.740000    -14.790000     -130.470000    0              -130.470000   
10.400000      -130.470000    18.110000      -124.670000    20.746966      -103.394126   
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
3              request_i     4              +Free+       
<Item>
<Position>
670.230000     448.610000    
</Position>
<BasePoints>
162.410000     -18.750000     0              0              105.476290     -5.354735     
138.010000     -7.030000      173.550000     -8.860000      219.515120     -11.219019    
299.480000     -15.280000    
</BasePoints>
</Item>
</Trans>
<Trans>
4              deny_i        2              +Free+       
<Item>
<Position>
969.710000     433.330000    
</Position>
<BasePoints>
-299.480000    -172.910000    0              0              -56.979646     -56.472143    
-92.980000     -88.860000     -142.200000    -125.920000    -194.010000    -143.050000   
-298.080000    -177.470000    -424.940000    -166.600000    -525.016979    -149.021846   
-610.490000    -130.550000   
</BasePoints>
</Item>
</Trans>
<Trans>
4              accept_i      5              +Free+       
<Item>
<Position>
969.710000     433.330000    
</Position>
<BasePoints>
133.590000     -7.640000      0              0              80.253422      0             
109.890000     0              143.290000     0              186.931355     0             
254.890000     0             
</BasePoints>
</Item>
</Trans>
<Trans>
5              tin_k         6              +Free+       
<Item>
<Position>
1224.600000    433.330000    
</Position>
<BasePoints>
115.800000     38.200000      0              0              58.884822      33.664153     
69.900000      38.720000      81.500000      43.090000      92.900000      45.840000     
113.800000     50.870000      136.600000     53.180000      172.959796     52.993331     
265.900000     47.230000     
</BasePoints>
</Item>
</Trans>
<Trans>
5              tout_k        6              +Free+       
<Item>
<Position>
1224.600000    433.330000    
</Position>
<BasePoints>
115.800000     7.640000       0              0              66.838083      11.753940     
95.300000      16.840000      128.800000     22.860000      174.137941     30.976494     
265.900000     47.230000     
</BasePoints>
</Item>
</Trans>
<Trans>
5              work_i        7              +Free+       
<Item>
<Position>
1224.600000    433.330000    
</Position>
<BasePoints>
115.800000     -114.580000    0              0              53.191559      -42.085082    
87.900000      -70.300000     134.100000     -107.850000    186.037095     -149.912299   
265.900000     -213.890000   
</BasePoints>
</Item>
</Trans>
<Trans>
6              broadcast     6              +Free+       
<Item>
<Position>
1490.500000    480.560000    
</Position>
<BasePoints>
0              -125.850000    0              0              -26.595780     -89.339905    
-23.400000     -106.420000    -14.500000     -118.210000    0              -118.210000   
10.200000      -118.210000    17.700000      -112.380000    19.689250      -91.038499    
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
6              work_i        8              +Free+       
<Item>
<Position>
1490.500000    480.560000    
</Position>
<BasePoints>
178.600000     -20.140000     0              0              93.190740      -4.714591     
137.400000     -6.980000      191.500000     -9.760000      254.910855     -13.002637    
381.700000     -19.450000    
</BasePoints>
</Item>
</Trans>
<Trans>
7              tin_k         8              +Free+       
<Item>
<Position>
1490.500000    219.440000    
</Position>
<BasePoints>
178.600000     78.480000      0              0              86.560795      54.327109     
138.700000     87.550000      205.400000     130.130000     274.558151     174.112357    
381.700000     241.670000    
</BasePoints>
</Item>
</Trans>
<Trans>
7              tout_k        8              +Free+       
<Item>
<Position>
1490.500000    219.440000    
</Position>
<BasePoints>
178.600000     154.870000     0              0              70.424030      73.960173     
81.800000      87.500000      92.700000      101.480000     102.200000     115.280000    
115.700000     134.850000     109            147.320000     127.200000     162.500000    
160            189.740000     202.100000     207.630000     256.347151     221.573893    
381.700000     241.670000    
</BasePoints>
</Item>
</Trans>
<Trans>
7              goal_reached_i  9              +Free+       
<Item>
<Position>
1490.500000    219.440000    
</Position>
<BasePoints>
178.600000     -40.970000     0              0              101.103806     -14.971332    
150.800000     -22.440000     210.600000     -31.430000     276.568783     -41.319751    
381.700000     -56.940000    
</BasePoints>
</Item>
</Trans>
<Trans>
8              broadcast     8              +Free+       
<Item>
<Position>
1872.200000    461.110000    
</Position>
<BasePoints>
0              -159.360000    0              0              -34.333431     -122.148937   
-28.800000     -139.770000    -17.400000     -151.720000    0              -151.720000   
12.800000      -151.720000    22.400000      -145.280000    26.425242      -124.143391   
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
8              goal_reached_i  10             +Free+       
<Item>
<Position>
1872.200000    461.110000    
</Position>
<BasePoints>
203.100000     -10.420000     0              0              126.925837     -1.276238     
170.800000     -1.730000      220.000000     -2.230000      279.448839     -2.836267     
411.200000     -4.170000     
</BasePoints>
</Item>
</Trans>
<Trans>
9              tin_k         10             +Free+       
<Item>
<Position>
1872.200000    162.500000    
</Position>
<BasePoints>
203.100000     104.860000     0              0              86.660405      61.489044     
146.500000     104.600000     226.500000     162.270000     304.004688     218.100405    
411.200000     294.440000    
</BasePoints>
</Item>
</Trans>
<Trans>
9              tout_k        10             +Free+       
<Item>
<Position>
1872.200000    162.500000    
</Position>
<BasePoints>
203.100000     195.140000     0              0              83.389701      66.204258     
99.700000      82.510000      115.100000     100.490000     126.800000     119.440000    
147            152.400000     125.100000     174.760000     151.800000     202.780000    
182.500000     235.100000     225.400000     256.040000     281.330241     272.015340    
411.200000     294.440000    
</BasePoints>
</Item>
</Trans>
<Trans>
9              pick_box_i    11             +Free+       
<Item>
<Position>
1872.200000    162.500000    
</Position>
<BasePoints>
203.100000     -7.640000      0              0              106.183899     0             
165.700000     0              239.300000     0              313.775714     0             
411.200000     0             
</BasePoints>
</Item>
</Trans>
<Trans>
10             broadcast     10             +Free+       
<Item>
<Position>
2283.400000    456.940000    
</Position>
<BasePoints>
0              -164.270000    0              0              -34.433528     -127.333986   
-28.800000     -144.840000    -17.300000     -156.630000    0              -156.630000   
12.600000      -156.630000    22.200000      -150.270000    26.367780      -128.990072   
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
10             pick_box_i    12             +Free+       
<Item>
<Position>
2283.400000    456.940000    
</Position>
<BasePoints>
208            -120.130000    0              0              123.857736     -44.373206    
168.200000     -62.770000     217.500000     -85.950000     259.400000     -112.500000   
275.400000     -122.660000    291.500000     -134.530000    317.357744     -156.720197   
407            -240.270000   
</BasePoints>
</Item>
</Trans>
<Trans>
11             tin_k         12             +Free+       
<Item>
<Position>
2283.400000    162.500000    
</Position>
<BasePoints>
208            42.360000      0              0              91.221572      33.853782     
112.500000     40.490000      135.100000     46.440000      156.600000     50            
193.100000     56.050000      233            58.520000      284.435239     58.607381     
407            54.170000     
</BasePoints>
</Item>
</Trans>
<Trans>
11             tout_k        12             +Free+       
<Item>
<Position>
2283.400000    162.500000    
</Position>
<BasePoints>
208            15.970000      0              0              96.605013      12.757456     
148.700000     19.750000      214.100000     28.500000      285.242245     38.010404     
407            54.170000     
</BasePoints>
</Item>
</Trans>
<Trans>
11             goal_reached_i  13             +Free+       
<Item>
<Position>
2283.400000    162.500000    
</Position>
<BasePoints>
208            85.420000      0              0              84.849872      47.918741     
137.100000     79.340000      204.300000     122.440000     259.400000     166.670000    
272.500000     177.170000     295.700000     199.460000     330.349146     234.054795    
407            311.110000    
</BasePoints>
</Item>
</Trans>
<Trans>
12             broadcast     12             +Free+       
<Item>
<Position>
2690.400000    216.670000    
</Position>
<BasePoints>
0              -155.280000    0              0              -33.434602     -117.926337   
-28.300000     -135.745000    -17.100000     -147.641000    0              -147.641000   
12.300000      -147.641000    21.600000      -141.496000    25.267781      -119.960842   
0              0             
</BasePoints>
</Item>
</Trans>
<Trans>
12             goal_reached_i  14             +Free+       
<Item>
<Position>
2690.400000    216.670000    
</Position>
<BasePoints>
199.000000     3.470000       0              0              122.538769     7.856839      
166.500000     10.710000      216.100000     13.930000      275.740810     17.789257     
409.500000     26.390000     
</BasePoints>
</Item>
</Trans>
<Trans>
13             drop_box_i    1              +Free+       
<Item>
<Position>
2690.400000    473.610000    
</Position>
<BasePoints>
-1465.800000   146.530000     0              0              -92.890448     56.805312     
-172.300000    100.710000     -292.400000    154.170000     -405.600000    154.170000    
-2332.570000   154.170000     -2332.570000   154.170000     -2332.570000   154.170000    
-2441.290000   154.170000     -2511.920000   31.120000      -2552.171289   -64.724686    
-2574.670000   -123.610000   
</BasePoints>
</Item>
</Trans>
<Trans>
13             tin_k         14             +Free+       
<Item>
<Position>
2690.400000    473.610000    
</Position>
<BasePoints>
199.000000     -143.750000    0              0              95.106487      -53.127762    
150.100000     -84.290000     220.100000     -123.990000    292.634258     -165.023965   
409.500000     -230.550000   
</BasePoints>
</Item>
</Trans>
<Trans>
13             tout_k        14             +Free+       
<Item>
<Position>
2690.400000    473.610000    
</Position>
<BasePoints>
199.000000     -75.690000     0              0              107.467117     -16.045507    
154.100000     -26.300000     207.200000     -42.520000     250.400000     -68.050000    
273.100000     -81.420000     294.600000     -98.890000     322.968236     -127.918249   
409.500000     -230.550000   
</BasePoints>
</Item>
</Trans>
<Trans>
14             drop_box_i    2              +Free+       
<Item>
<Position>
3099.900000    243.060000    
</Position>
<BasePoints>
-1609.400000   -235.421100    0              0              -98.395809     -91.429025    
-175.800000    -154.707000    -290.700000    -227.782000    -408.100000    -227.782000   
-2431.060000   -227.782000    -2431.060000   -227.782000    -2431.060000   -227.782000   
-2548.760000   -227.782000    -2640.230000   -114.300000    -2698.908245   -17.160268    
-2740.680000   59.720000     
</BasePoints>
</Item>
</Trans>
<Trans>
14             broadcast     14             +Free+       
<Item>
<Position>
3099.900000    243.060000    
</Position>
<BasePoints>
0              -166.728000    0              0              -34.532455     -129.586530   
-28.800000     -147.397000    -17.300000     -159.089000    0              -159.089000   
12.700000      -159.089000    22.300000      -152.783000    26.552170      -131.359484   
0              0             
</BasePoints>
</Item>
</Trans>
</TransRel>
</GraphData>
</VioModels>
<VioLayout>
0             =AAAA/wAAAAAAAAADAAADvgAAA74AAAAAAQAAAAQBAAAAAQ===  0              150            958            0.464370       0             
958           
</VioLayout>
</VioSystem>
</Value>
</Variable>
<Variable>
R_monolitic   System        +Visual+     
<Value>
<VioSystem>
<Generator name="R_monolitic" ftype="System">

<Alphabet>
<Event name="tin_k">
<Controllable/>
</Event>
<Event name="broadcast"/>
<Event name="request_i">
<Controllable/>
</Event>
<Event name="accept_i">
<Controllable/>
</Event>
<Event name="tout_k">
<Controllable/>
</Event>
<Event name="deny_i">
<Controllable/>
</Event>
<Event name="work_i">
<Controllable/>
</Event>
<Event name="goal_reached_i"/>
<Event name="pick_box_i"/>
<Event name="drop_box_i"/>
</Alphabet>

<StateSet>
<State id="1" name="Idle|Empty|Idle|Idle|Idle">
<Initial/>
<Marked/>
</State>
<State id="2" name="Broadcast|Full|Broadcast|Idle|Idle"/>
<State id="3" name="Broadcast|Full|Broadcast|Idle|Request"/>
<State id="4" name="Answer|Full|Answer|Idle|Wait"/>
<State id="5" name="Idle|Full|Idle|Idle|Work"/>
<State id="6" name="Broadcast|Empty|Broadcast|Idle|Work"/>
<State id="7" name="Idle|Full|Idle|ToStart|WorkDone"/>
<State id="8" name="Broadcast|Empty|Broadcast|ToStart|WorkDone"/>
<State id="9" name="Idle|Full|Idle|PickBox|WorkDone"/>
<State id="10" name="Broadcast|Empty|Broadcast|PickBox|WorkDone"/>
<State id="11" name="Idle|Full|Idle|ToEnd|WorkDone"/>
<State id="12" name="Broadcast|Empty|Broadcast|ToEnd|WorkDone"/>
<State id="13" name="Idle|Full|Idle|DropBox|WorkDone"/>
<State id="14" name="Broadcast|Empty|Broadcast|DropBox|WorkDone"/>
<State id="15" name="Idle|Full|Idle|Idle|Idle">
<Marked/>
</State>
<State id="16" name="Broadcast|Empty|Broadcast|Idle|Idle"/>
<State id="17" name="Broadcast|Empty|Broadcast|Idle|Request"/>
<State id="18" name="Answer|Empty|Answer|Idle|Wait"/>
<State id="19" name="Idle|Empty|Idle|Idle|Work"/>
<State id="20" name="Broadcast|Full|Broadcast|Idle|Work"/>
<State id="21" name="Idle|Empty|Idle|ToStart|WorkDone"/>
<State id="22" name="Broadcast|Full|Broadcast|ToStart|WorkDone"/>
<State id="23" name="Idle|Empty|Idle|PickBox|WorkDone"/>
<State id="24" name="Broadcast|Full|Broadcast|PickBox|WorkDone"/>
<State id="25" name="Idle|Empty|Idle|ToEnd|WorkDone"/>
<State id="26" name="Broadcast|Full|Broadcast|ToEnd|WorkDone"/>
<State id="27" name="Idle|Empty|Idle|DropBox|WorkDone"/>
<State id="28" name="Broadcast|Full|Broadcast|DropBox|WorkDone"/>
</StateSet>

<TransitionRelation>
<Transition x1="1" event="tin_k" x2="2"/>
<Transition x1="2" event="broadcast" x2="3"/>
<Transition x1="3" event="broadcast" x2="3"/>
<Transition x1="3" event="request_i" x2="4"/>
<Transition x1="4" event="accept_i" x2="5"/>
<Transition x1="4" event="deny_i" x2="2"/>
<Transition x1="5" event="tout_k" x2="6"/>
<Transition x1="5" event="work_i" x2="7"/>
<Transition x1="6" event="broadcast" x2="6"/>
<Transition x1="6" event="work_i" x2="8"/>
<Transition x1="7" event="tout_k" x2="8"/>
<Transition x1="7" event="goal_reached_i" x2="9"/>
<Transition x1="8" event="broadcast" x2="8"/>
<Transition x1="8" event="goal_reached_i" x2="10"/>
<Transition x1="9" event="tout_k" x2="10"/>
<Transition x1="9" event="pick_box_i" x2="11"/>
<Transition x1="10" event="broadcast" x2="10"/>
<Transition x1="10" event="pick_box_i" x2="12"/>
<Transition x1="11" event="tout_k" x2="12"/>
<Transition x1="11" event="goal_reached_i" x2="13"/>
<Transition x1="12" event="broadcast" x2="12"/>
<Transition x1="12" event="goal_reached_i" x2="14"/>
<Transition x1="13" event="tout_k" x2="14"/>
<Transition x1="13" event="drop_box_i" x2="15"/>
<Transition x1="14" event="broadcast" x2="14"/>
<Transition x1="14" event="drop_box_i" x2="16"/>
<Transition x1="15" event="tout_k" x2="16"/>
<Transition x1="16" event="broadcast" x2="17"/>
<Transition x1="17" event="broadcast" x2="17"/>
<Transition x1="17" event="request_i" x2="18"/>
<Transition x1="18" event="accept_i" x2="19"/>
<Transition x1="18" event="deny_i" x2="16"/>
<Transition x1="19" event="tin_k" x2="20"/>
<Transition x1="19" event="work_i" x2="21"/>
<Transition x1="20" event="broadcast" x2="20"/>
<Transition x1="20" event="work_i" x2="22"/>
<Transition x1="21" event="tin_k" x2="22"/>
<Transition x1="21" event="goal_reached_i" x2="23"/>
<Transition x1="22" event="broadcast" x2="22"/>
<Transition x1="22" event="goal_reached_i" x2="24"/>
<Transition x1="23" event="tin_k" x2="24"/>
<Transition x1="23" event="pick_box_i" x2="25"/>
<Transition x1="24" event="broadcast" x2="24"/>
<Transition x1="24" event="pick_box_i" x2="26"/>
<Transition x1="25" event="tin_k" x2="26"/>
<Transition x1="25" event="goal_reached_i" x2="27"/>
<Transition x1="26" event="broadcast" x2="26"/>
<Transition x1="26" event="goal_reached_i" x2="28"/>
<Transition x1="27" event="tin_k" x2="28"/>
<Transition x1="27" event="drop_box_i" x2="1"/>
<Transition x1="28" event="broadcast" x2="28"/>
<Transition x1="28" event="drop_box_i" x2="2"/>
</TransitionRelation>

</Generator>
<VioModels>
<TransitionList>
=AAAANAEAAAABAAAACgB0AGkAbgBfAGsAAAACAQAAAAQAAAAQAGEAYwBjAGUAcAB0AF8AaQAAAAUBAAAAAgAAABIAYgByAG8AYQBkAGMAYQBzAHQAAAADAQAAAAMAAAASAGIAcgBvAGEAZABjAGEAcwB0AAAAAwEAAAADAAAAEgByAGUAcQB1AGUAcwB0AF8AaQAAAAQBAAAACAAAABIAYgByAG8AYQBkAGMAYQBzAHQAAAAIAQAAAAwAAAAcAGcAbwBhAGwAXwByAGUAYQBjAGgAZQBkAF8AaQAAAA4BAAAAEQAAABIAYgByAG8AYQBkAGMAYQBzAHQAAAARAQAAABQAAAASAGIAcgBvAGEAZABjAGEAcwB0AAAAFAEAAAAUAAAADAB3AG8AcgBrAF8AaQAAABYBAAAAFQAAABwAZwBvAGEAbABfAHIAZQBhAGMAaABlAGQAXwBpAAAAFwEAAAAWAAAAEgBiAHIAbwBhAGQAYwBhAHMAdAAAABYBAAAAFgAAABwAZwBvAGEAbABfAHIAZQBhAGMAaABlAGQAXwBpAAAAGAEAAAAXAAAAFABwAGkAYwBrAF8AYgBvAHgAXwBpAAAAGQEAAAAYAAAAEgBiAHIAbwBhAGQAYwBhAHMAdAAAABgBAAAAGAAAABQAcABpAGMAawBfAGIAbwB4AF8AaQAAABoBAAAAGQAAABwAZwBvAGEAbABfAHIAZQBhAGMAaABlAGQAXwBpAAAAGwEAAAAaAAAAEgBiAHIAbwBhAGQAYwBhAHMAdAAAABoBAAAAGgAAABwAZwBvAGEAbABfAHIAZQBhAGMAaABlAGQAXwBpAAAAHAEAAAAcAAAAEgBiAHIAbwBhAGQAYwBhAHMAdAAAABwBAAAABAAAAAwAZABlAG4AeQBfAGkAAAACAQAAAAUAAAAMAHQAbwB1AHQAXwBrAAAABgEAAAAFAAAADAB3AG8AcgBrAF8AaQAAAAcBAAAABgAAABIAYgByAG8AYQBkAGMAYQBzAHQAAAAGAQAAAAYAAAAMAHcAbwByAGsAXwBpAAAACAEAAAAHAAAADAB0AG8AdQB0AF8AawAAAAgBAAAABwAAABwAZwBvAGEAbABfAHIAZQBhAGMAaABlAGQAXwBpAAAACQEAAAAIAAAAHABnAG8AYQBsAF8AcgBlAGEAYwBoAGUAZABfAGkAAAAKAQAAAAkAAAAMAHQAbwB1AHQAXwBrAAAACgEAAAAJAAAAFABwAGkAYwBrAF8AYgBvAHgAXwBpAAAACwEAAAAKAAAAEgBiAHIAbwBhAGQAYwBhAHMAdAAAAAoBAAAACgAAABQAcABpAGMAawBfAGIAbwB4AF8AaQAAAAwBAAAACwAAAAwAdABvAHUAdABfAGsAAAAMAQAAAAsAAAAcAGcAbwBhAGwAXwByAGUAYQBjAGgAZQBkAF8AaQAAAA0BAAAADAAAABIAYgByAG8AYQBkAGMAYQBzAHQAAAAMAQAAAA0AAAAMAHQAbwB1AHQAXwBrAAAADgEAAAANAAAAFABkAHIAbwBwAF8AYgBvAHgAXwBpAAAADwEAAAAOAAAAEgBiAHIAbwBhAGQAYwBhAHMAdAAAAA4BAAAADgAAABQAZAByAG8AcABfAGIAbwB4AF8AaQAAABABAAAADwAAAAwAdABvAHUAdABfAGsAAAAQAQAAABAAAAASAGIAcgBvAGEAZABjAGEAcwB0AAAAEQEAAAARAAAAEgByAGUAcQB1AGUAcwB0AF8AaQAAABIBAAAAEgAAABAAYQBjAGMAZQBwAHQAXwBpAAAAEwEAAAASAAAADABkAGUAbgB5AF8AaQAAABABAAAAEwAAAAoAdABpAG4AXwBrAAAAFAEAAAATAAAADAB3AG8AcgBrAF8AaQAAABUBAAAAFQAAAAoAdABpAG4AXwBrAAAAFgEAAAAXAAAACgB0AGkAbgBfAGsAAAAYAQAAABkAAAAKAHQAaQBuAF8AawAAABoBAAAAGwAAAAoAdABpAG4AXwBrAAAAHAEAAAAbAAAAFABkAHIAbwBwAF8AYgBvAHgAXwBpAAAAAQEAAAAcAAAAFABkAHIAbwBwAF8AYgBvAHgAXwBpAAAAAg=== </TransitionList>
<StateList>
=AAAAHAIAAAABAgAAAAICAAAAAwIAAAAEAgAAAAUCAAAABgIAAAAHAgAAAAgCAAAACQIAAAAKAgAAAAsCAAAADAIAAAANAgAAAA4CAAAADwIAAAAQAgAAABECAAAAEgIAAAATAgAAABQCAAAAFQIAAAAWAgAAABcCAAAAGAIAAAAZAgAAABoCAAAAGwIAAAAc= </StateList>
<EventList>
=AAAACgMAAAAKAHQAaQBuAF8AawMAAAASAGIAcgBvAGEAZABjAGEAcwB0AwAAABIAcgBlAHEAdQBlAHMAdABfAGkDAAAAEABhAGMAYwBlAHAAdABfAGkDAAAADAB0AG8AdQB0AF8AawMAAAAMAGQAZQBuAHkAXwBpAwAAAAwAdwBvAHIAawBfAGkDAAAAHABnAG8AYQBsAF8AcgBlAGEAYwBoAGUAZABfAGkDAAAAFABwAGkAYwBrAF8AYgBvAHgAXwBpAwAAABQAZAByAG8AcABfAGIAbwB4AF8AaQ=== </EventList>
<GraphData>
<States/>
<TransRel/>
</GraphData>
</VioModels>
<VioLayout>
0             =AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAABmZg===  0              150            472            0.371278       0             
472           
</VioLayout>
</VioSystem>
</Value>
</Variable>
<Variable>
Is_monolitic_trim  Boolean      
<Value>
<Boolean>
true         
</Boolean>
</Value>
</Variable>
</VariablePool>

<Functions/>
<Script>
<AlphabetExtract>
AlphabetExtract_3  0             
<Parameters>
G_TaskTransmitter  E_TaskTransmitter 
</Parameters>
<Options>
"Clear Arguments"  0             
</Options>
</AlphabetExtract>
<InvProject>
InvProject_2  1             
<Parameters>
Hs_Storage_k_TaskTransmitter  E_TaskTransmitter  Hsinv_Storage_k_TaskTransmitter 
</Parameters>
<Options>
"Clear Arguments"  0              "Minimal Realisation"  0              "Clear State Names"  0             
</Options>
</InvProject>
<SupConNB>
SupConNB_3    0             
<Parameters>
G_TaskTransmitter  Hsinv_Storage_k_TaskTransmitter  Rl_Storage_k_TaskTransmitter 
</Parameters>
<Options>
"Clear Arguments"  0              "Minimal Realisation"  0              "Clear State Names"  0             
</Options>
</SupConNB>
<Parallel>
Parallel_9    0             
<Parameters>
G_TaskTransmitter  G_Robot_i     Gl_TaskTrasmitter_Robot_i 
</Parameters>
<Options>
"Clear Arguments"  0              "Minimal Realisation"  0              "Clear State Names"  0             
</Options>
</Parallel>
<AlphabetExtract>
AlphabetExtract_9  0             
<Parameters>
Gl_TaskTrasmitter_Robot_i  E_TaskTransmitter_Robot_i 
</Parameters>
<Options>
"Clear Arguments"  0             
</Options>
</AlphabetExtract>
<InvProject>
InvProject_5  1             
<Parameters>
Hs_Request_i_TaskTransmitter_Robot_i  E_TaskTransmitter_Robot_i  Hsinv_Request_i_TaskTransmitter_Robot_i 
</Parameters>
<Options>
"Clear Arguments"  0              "Minimal Realisation"  0              "Clear State Names"  0             
</Options>
</InvProject>
<SupConNB>
SupConNB_6    0             
<Parameters>
Gl_TaskTrasmitter_Robot_i  Hsinv_Request_i_TaskTransmitter_Robot_i  Rl_Request_i_TaskTransmitter_Robot_i 
</Parameters>
<Options>
"Clear Arguments"  0              "Minimal Realisation"  0              "Clear State Names"  0             
</Options>
</SupConNB>
<Parallel>
Parallel_7    0             
<Parameters>
Rl_Storage_k_TaskTransmitter  Rl_Request_i_TaskTransmitter_Robot_i  R_monolitic  
</Parameters>
<Options>
"Clear Arguments"  0              "Minimal Realisation"  0              "Clear State Names"  0             
</Options>
</Parallel>
<IsTrim>
IsTrim_10     0             
<Parameters>
R_monolitic   Is_monolitic_trim 
</Parameters>
<Options>
"Clear Arguments"  0             
</Options>
</IsTrim>
</Script>

<Simulator>

<DevFile>
""           
</DevFile>
<SimEvents>
tin_k        
<Priority>
0             
</Priority>
broadcast    
<Priority>
0             
</Priority>
request_i    
<Priority>
0             
</Priority>
accept_i     
<Priority>
0             
</Priority>
tout_k       
<Priority>
0             
</Priority>
deny_i       
<Priority>
0             
</Priority>
work_i       
<Priority>
0             
</Priority>
goal_reached_i 
<Priority>
0             
</Priority>
pick_box_i   
<Priority>
0             
</Priority>
drop_box_i   
<Priority>
0             
</Priority>
</SimEvents>
<Conditions/>
</Simulator>
<GuiState/>
</Project>
