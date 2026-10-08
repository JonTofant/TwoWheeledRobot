# Frozen recovery results

These are development adaptations evaluated under the corrected sensor-timing contract. Original models are evaluated under that same corrected simulator; these figures do not reproduce their historical training conditions or rank their physical performance. Cohorts share the same terrain layout but have independent initial-state and motor draws. No intermediate checkpoint selection or gate relaxation was used.

Falls, survival and coverage precede alive-masked tracking. There are 64 robots per scenario, 11 scenarios and 15 s per trial. Each split has 704 initial episodes. Current RMS is simulated post-actuator-model effective current; command change is the logical network-output change. Neither is a direct measurement of hardware torque.

## Attempt 1

Final checkpoint: `model_874.pt`. SHA256: `c5c0ac0c3c7d76f07016f1718cf93a76f7f60733e75ba51bb6cb6e55890ea8d8`. Status: **completed_not_qualified**.

### Validation — base seed 4001

| Model | Falls / 704 | Mean survival (s) | Minimum sample coverage (%) | Gate failures |
|---|---:|---:|---:|---|
| Original GRU 775 | 0 | 15.000 | 100.0 | drive_forward_slow: rms_vel_err_mps 0.207 > 0.120; drive_backward_slow: rms_vel_err_mps 0.167 > 0.120; payload_while_still: world_drift_m 0.680 > 0.500 |
| Earlier recovery 924 | 0 | 15.000 | 100.0 | drive_forward_slow: rms_vel_err_mps 0.151 > 0.120; drive_backward_slow: rms_vel_err_mps 0.162 > 0.120; payload_while_still: world_drift_m 0.508 > 0.500 |
| Failed fresh pilot 1150 | 0 | 15.000 | 100.0 | None |
| Adaptation final | 0 | 15.000 | 100.0 | drive_forward_slow: rms_vel_err_mps 0.136 > 0.120; drive_backward_slow: rms_vel_err_mps 0.184 > 0.120; drive_backward: rms_vel_err_mps 0.205 > 0.195 |

No invalid-state or physics-broken terminations occurred in this split. Coverage is the smallest scored-sample fraction across scenarios, after the common settling exclusion.

| Model / scenario | Falls (%) | Survival (s) | Velocity error RMS (m/s) | Yaw-rate error RMS (rad/s) | Final world drift (m) | Effective current RMS (A) | Command change (A/tick) |
|---|---:|---:|---:|---:|---:|---:|---:|
| Original GRU 775 / station_keeping | 0.0 | 15.000 | 0.0648 | 0.0541 | 0.3692 | 0.0930 | 0.0855 |
| Original GRU 775 / drive_forward_slow | 0.0 | 15.000 | 0.2071 | 0.1177 | 2.9384 | 0.1597 | 0.1135 |
| Original GRU 775 / drive_backward_slow | 0.0 | 15.000 | 0.1666 | 0.0904 | 2.2206 | 0.1871 | 0.1802 |
| Original GRU 775 / drive_forward | 0.0 | 15.000 | 0.1811 | 0.0948 | 4.8142 | 0.1500 | 0.0977 |
| Original GRU 775 / drive_backward | 0.0 | 15.000 | 0.1904 | 0.1214 | 4.3414 | 0.1869 | 0.2009 |
| Original GRU 775 / turn_in_place | 0.0 | 15.000 | 0.1099 | 0.0893 | 0.3257 | 0.1379 | 0.1050 |
| Original GRU 775 / drive_and_turn | 0.0 | 15.000 | 0.1404 | 0.1087 | 1.8099 | 0.1101 | 0.0758 |
| Original GRU 775 / push_while_still | 0.0 | 15.000 | 0.0924 | 0.0667 | 0.4872 | 0.1170 | 0.0933 |
| Original GRU 775 / push_while_driving | 0.0 | 15.000 | 0.1834 | 0.0955 | 4.5872 | 0.1398 | 0.0881 |
| Original GRU 775 / payload_while_still | 0.0 | 15.000 | 0.1085 | 0.0718 | 0.6801 | 0.1340 | 0.1130 |
| Original GRU 775 / payload_while_driving | 0.0 | 15.000 | 0.2085 | 0.0988 | 4.9880 | 0.1476 | 0.0982 |
| Earlier recovery 924 / station_keeping | 0.0 | 15.000 | 0.0553 | 0.0482 | 0.0827 | 0.0778 | 0.0616 |
| Earlier recovery 924 / drive_forward_slow | 0.0 | 15.000 | 0.1513 | 0.0909 | 2.3716 | 0.0942 | 0.0607 |
| Earlier recovery 924 / drive_backward_slow | 0.0 | 15.000 | 0.1624 | 0.0891 | 2.7293 | 0.1177 | 0.1065 |
| Earlier recovery 924 / drive_forward | 0.0 | 15.000 | 0.1539 | 0.0948 | 4.0816 | 0.0928 | 0.0537 |
| Earlier recovery 924 / drive_backward | 0.0 | 15.000 | 0.1585 | 0.1082 | 4.5913 | 0.1225 | 0.1213 |
| Earlier recovery 924 / turn_in_place | 0.0 | 15.000 | 0.0724 | 0.0839 | 0.2006 | 0.0570 | 0.0460 |
| Earlier recovery 924 / drive_and_turn | 0.0 | 15.000 | 0.1049 | 0.1306 | 1.4607 | 0.0717 | 0.0421 |
| Earlier recovery 924 / push_while_still | 0.0 | 15.000 | 0.0671 | 0.0568 | 0.0923 | 0.0736 | 0.0575 |
| Earlier recovery 924 / push_while_driving | 0.0 | 15.000 | 0.1605 | 0.0949 | 4.2193 | 0.0959 | 0.0539 |
| Earlier recovery 924 / payload_while_still | 0.0 | 15.000 | 0.0901 | 0.0736 | 0.5080 | 0.0907 | 0.0792 |
| Earlier recovery 924 / payload_while_driving | 0.0 | 15.000 | 0.1847 | 0.0932 | 4.4114 | 0.0921 | 0.0505 |
| Failed fresh pilot 1150 / station_keeping | 0.0 | 15.000 | 0.0603 | 0.0511 | 0.0579 | 0.1498 | 0.2638 |
| Failed fresh pilot 1150 / drive_forward_slow | 0.0 | 15.000 | 0.0933 | 0.0673 | 2.0111 | 0.1474 | 0.2185 |
| Failed fresh pilot 1150 / drive_backward_slow | 0.0 | 15.000 | 0.0951 | 0.0645 | 1.9136 | 0.1638 | 0.2405 |
| Failed fresh pilot 1150 / drive_forward | 0.0 | 15.000 | 0.0901 | 0.0690 | 4.5855 | 0.1569 | 0.2329 |
| Failed fresh pilot 1150 / drive_backward | 0.0 | 15.000 | 0.0848 | 0.0658 | 4.0655 | 0.1269 | 0.1512 |
| Failed fresh pilot 1150 / turn_in_place | 0.0 | 15.000 | 0.0710 | 0.0779 | 0.1839 | 0.1586 | 0.2876 |
| Failed fresh pilot 1150 / drive_and_turn | 0.0 | 15.000 | 0.0731 | 0.0990 | 1.5109 | 0.1405 | 0.2433 |
| Failed fresh pilot 1150 / push_while_still | 0.0 | 15.000 | 0.0632 | 0.0527 | 0.0943 | 0.1315 | 0.2277 |
| Failed fresh pilot 1150 / push_while_driving | 0.0 | 15.000 | 0.1061 | 0.0762 | 4.6846 | 0.1854 | 0.2871 |
| Failed fresh pilot 1150 / payload_while_still | 0.0 | 15.000 | 0.0673 | 0.0531 | 0.2222 | 0.1192 | 0.2063 |
| Failed fresh pilot 1150 / payload_while_driving | 0.0 | 15.000 | 0.1134 | 0.0738 | 5.1893 | 0.1437 | 0.2067 |
| Adaptation final / station_keeping | 0.0 | 15.000 | 0.0419 | 0.0489 | 0.0631 | 0.0558 | 0.0542 |
| Adaptation final / drive_forward_slow | 0.0 | 15.000 | 0.1360 | 0.0971 | 2.0973 | 0.0895 | 0.0650 |
| Adaptation final / drive_backward_slow | 0.0 | 15.000 | 0.1843 | 0.1218 | 3.1160 | 0.1751 | 0.1924 |
| Adaptation final / drive_forward | 0.0 | 15.000 | 0.1574 | 0.0962 | 4.0570 | 0.0916 | 0.0579 |
| Adaptation final / drive_backward | 0.0 | 15.000 | 0.2055 | 0.1691 | 5.4133 | 0.2581 | 0.3416 |
| Adaptation final / turn_in_place | 0.0 | 15.000 | 0.0788 | 0.0904 | 0.2096 | 0.0639 | 0.0520 |
| Adaptation final / drive_and_turn | 0.0 | 15.000 | 0.1051 | 0.1310 | 1.4160 | 0.0658 | 0.0432 |
| Adaptation final / push_while_still | 0.0 | 15.000 | 0.0617 | 0.0674 | 0.0907 | 0.0744 | 0.0651 |
| Adaptation final / push_while_driving | 0.0 | 15.000 | 0.1649 | 0.0959 | 4.0634 | 0.0902 | 0.0576 |
| Adaptation final / payload_while_still | 0.0 | 15.000 | 0.0757 | 0.0768 | 0.3744 | 0.0903 | 0.0963 |
| Adaptation final / payload_while_driving | 0.0 | 15.000 | 0.1836 | 0.1033 | 4.6189 | 0.1053 | 0.0773 |

### Test — base seed 5001

| Model | Falls / 704 | Mean survival (s) | Minimum sample coverage (%) | Gate failures |
|---|---:|---:|---:|---|
| Original GRU 775 | 0 | 15.000 | 100.0 | drive_forward_slow: rms_vel_err_mps 0.171 > 0.120; drive_backward_slow: rms_vel_err_mps 0.154 > 0.120; push_while_still: world_drift_m 0.675 > 0.500; payload_while_still: world_drift_m 1.099 > 0.500 |
| Earlier recovery 924 | 0 | 15.000 | 100.0 | drive_forward_slow: rms_vel_err_mps 0.123 > 0.120; drive_backward_slow: rms_vel_err_mps 0.136 > 0.120 |
| Failed fresh pilot 1150 | 0 | 15.000 | 100.0 | None |
| Adaptation final | 0 | 15.000 | 100.0 | drive_backward_slow: rms_vel_err_mps 0.160 > 0.120 |

No invalid-state or physics-broken terminations occurred in this split. Coverage is the smallest scored-sample fraction across scenarios, after the common settling exclusion.

| Model / scenario | Falls (%) | Survival (s) | Velocity error RMS (m/s) | Yaw-rate error RMS (rad/s) | Final world drift (m) | Effective current RMS (A) | Command change (A/tick) |
|---|---:|---:|---:|---:|---:|---:|---:|
| Original GRU 775 / station_keeping | 0.0 | 15.000 | 0.0805 | 0.0611 | 0.4506 | 0.1153 | 0.0993 |
| Original GRU 775 / drive_forward_slow | 0.0 | 15.000 | 0.1712 | 0.0980 | 2.5234 | 0.1424 | 0.0968 |
| Original GRU 775 / drive_backward_slow | 0.0 | 15.000 | 0.1539 | 0.0901 | 1.9971 | 0.1890 | 0.1848 |
| Original GRU 775 / drive_forward | 0.0 | 15.000 | 0.1579 | 0.0906 | 4.5305 | 0.1230 | 0.0823 |
| Original GRU 775 / drive_backward | 0.0 | 15.000 | 0.1603 | 0.1257 | 4.5742 | 0.1929 | 0.2167 |
| Original GRU 775 / turn_in_place | 0.0 | 15.000 | 0.0922 | 0.0840 | 0.3186 | 0.1063 | 0.0843 |
| Original GRU 775 / drive_and_turn | 0.0 | 15.000 | 0.1312 | 0.1082 | 1.6777 | 0.1074 | 0.0755 |
| Original GRU 775 / push_while_still | 0.0 | 15.000 | 0.1104 | 0.0730 | 0.6746 | 0.1497 | 0.1212 |
| Original GRU 775 / push_while_driving | 0.0 | 15.000 | 0.1609 | 0.0890 | 4.5299 | 0.1284 | 0.0788 |
| Original GRU 775 / payload_while_still | 0.0 | 15.000 | 0.1528 | 0.1011 | 1.0991 | 0.1827 | 0.1625 |
| Original GRU 775 / payload_while_driving | 0.0 | 15.000 | 0.1956 | 0.1004 | 5.1207 | 0.1533 | 0.1029 |
| Earlier recovery 924 / station_keeping | 0.0 | 15.000 | 0.0479 | 0.0463 | 0.0812 | 0.0668 | 0.0547 |
| Earlier recovery 924 / drive_forward_slow | 0.0 | 15.000 | 0.1232 | 0.0851 | 2.0928 | 0.0749 | 0.0538 |
| Earlier recovery 924 / drive_backward_slow | 0.0 | 15.000 | 0.1364 | 0.0778 | 2.4378 | 0.0974 | 0.0938 |
| Earlier recovery 924 / drive_forward | 0.0 | 15.000 | 0.1300 | 0.0917 | 4.0825 | 0.0805 | 0.0495 |
| Earlier recovery 924 / drive_backward | 0.0 | 15.000 | 0.1239 | 0.0985 | 4.3854 | 0.0866 | 0.0702 |
| Earlier recovery 924 / turn_in_place | 0.0 | 15.000 | 0.0819 | 0.0879 | 0.2072 | 0.0717 | 0.0519 |
| Earlier recovery 924 / drive_and_turn | 0.0 | 15.000 | 0.0959 | 0.1322 | 1.3391 | 0.0784 | 0.0454 |
| Earlier recovery 924 / push_while_still | 0.0 | 15.000 | 0.0649 | 0.0551 | 0.0859 | 0.0768 | 0.0643 |
| Earlier recovery 924 / push_while_driving | 0.0 | 15.000 | 0.1351 | 0.0963 | 4.1092 | 0.0872 | 0.0496 |
| Earlier recovery 924 / payload_while_still | 0.0 | 15.000 | 0.0721 | 0.0680 | 0.3459 | 0.0837 | 0.0703 |
| Earlier recovery 924 / payload_while_driving | 0.0 | 15.000 | 0.1516 | 0.0967 | 4.7510 | 0.0901 | 0.0527 |
| Failed fresh pilot 1150 / station_keeping | 0.0 | 15.000 | 0.0563 | 0.0519 | 0.0524 | 0.1519 | 0.2524 |
| Failed fresh pilot 1150 / drive_forward_slow | 0.0 | 15.000 | 0.0891 | 0.0704 | 1.9489 | 0.1530 | 0.2438 |
| Failed fresh pilot 1150 / drive_backward_slow | 0.0 | 15.000 | 0.0891 | 0.0622 | 1.9512 | 0.1377 | 0.2024 |
| Failed fresh pilot 1150 / drive_forward | 0.0 | 15.000 | 0.0874 | 0.0726 | 4.5684 | 0.1661 | 0.2672 |
| Failed fresh pilot 1150 / drive_backward | 0.0 | 15.000 | 0.0763 | 0.0661 | 3.9893 | 0.1149 | 0.1443 |
| Failed fresh pilot 1150 / turn_in_place | 0.0 | 15.000 | 0.0677 | 0.0787 | 0.1684 | 0.1384 | 0.2517 |
| Failed fresh pilot 1150 / drive_and_turn | 0.0 | 15.000 | 0.0732 | 0.0934 | 1.4002 | 0.1438 | 0.2602 |
| Failed fresh pilot 1150 / push_while_still | 0.0 | 15.000 | 0.0629 | 0.0508 | 0.0805 | 0.1223 | 0.2156 |
| Failed fresh pilot 1150 / push_while_driving | 0.0 | 15.000 | 0.0954 | 0.0725 | 4.5027 | 0.1481 | 0.2207 |
| Failed fresh pilot 1150 / payload_while_still | 0.0 | 15.000 | 0.0680 | 0.0539 | 0.2393 | 0.1168 | 0.2064 |
| Failed fresh pilot 1150 / payload_while_driving | 0.0 | 15.000 | 0.1115 | 0.0732 | 5.1600 | 0.1340 | 0.2031 |
| Adaptation final / station_keeping | 0.0 | 15.000 | 0.0445 | 0.0510 | 0.0792 | 0.0595 | 0.0575 |
| Adaptation final / drive_forward_slow | 0.0 | 15.000 | 0.1172 | 0.0850 | 1.9438 | 0.0729 | 0.0564 |
| Adaptation final / drive_backward_slow | 0.0 | 15.000 | 0.1602 | 0.1133 | 2.9238 | 0.1674 | 0.2046 |
| Adaptation final / drive_forward | 0.0 | 15.000 | 0.1317 | 0.0886 | 3.9808 | 0.0734 | 0.0482 |
| Adaptation final / drive_backward | 0.0 | 15.000 | 0.1634 | 0.1490 | 5.0972 | 0.1867 | 0.2384 |
| Adaptation final / turn_in_place | 0.0 | 15.000 | 0.0823 | 0.0951 | 0.2319 | 0.0691 | 0.0550 |
| Adaptation final / drive_and_turn | 0.0 | 15.000 | 0.0871 | 0.1179 | 1.3371 | 0.0552 | 0.0365 |
| Adaptation final / push_while_still | 0.0 | 15.000 | 0.0599 | 0.0606 | 0.1185 | 0.0686 | 0.0623 |
| Adaptation final / push_while_driving | 0.0 | 15.000 | 0.1330 | 0.0988 | 4.0479 | 0.0804 | 0.0512 |
| Adaptation final / payload_while_still | 0.0 | 15.000 | 0.0715 | 0.0764 | 0.3271 | 0.0921 | 0.1013 |
| Adaptation final / payload_while_driving | 0.0 | 15.000 | 0.1567 | 0.1022 | 4.3728 | 0.0909 | 0.0632 |

## Attempt 2

Final checkpoint: `model_1249.pt`. SHA256: `2661e25bc0811fb67cfc8e1cfeb70017224c6e01978e997eb1aab7e520c2aee0`. Status: **completed_not_qualified**.

### Validation — base seed 6001

| Model | Falls / 704 | Mean survival (s) | Minimum sample coverage (%) | Gate failures |
|---|---:|---:|---:|---|
| Original GRU 775 | 0 | 15.000 | 100.0 | station_keeping: world_drift_m 0.630 > 0.500; drive_forward_slow: rms_vel_err_mps 0.181 > 0.120; drive_backward_slow: rms_vel_err_mps 0.158 > 0.120; push_while_still: world_drift_m 0.571 > 0.500; payload_while_still: world_drift_m 0.699 > 0.500 |
| First adaptation 874 | 0 | 15.000 | 100.0 | drive_forward_slow: rms_vel_err_mps 0.126 > 0.120; drive_backward_slow: rms_vel_err_mps 0.165 > 0.120 |
| Failed fresh pilot 1150 | 0 | 15.000 | 100.0 | None |
| Adaptation final | 0 | 15.000 | 100.0 | None |

No invalid-state or physics-broken terminations occurred in this split. Coverage is the smallest scored-sample fraction across scenarios, after the common settling exclusion.

| Model / scenario | Falls (%) | Survival (s) | Velocity error RMS (m/s) | Yaw-rate error RMS (rad/s) | Final world drift (m) | Effective current RMS (A) | Command change (A/tick) |
|---|---:|---:|---:|---:|---:|---:|---:|
| Original GRU 775 / station_keeping | 0.0 | 15.000 | 0.0939 | 0.0659 | 0.6300 | 0.1256 | 0.1067 |
| Original GRU 775 / drive_forward_slow | 0.0 | 15.000 | 0.1813 | 0.0999 | 2.7742 | 0.1449 | 0.1055 |
| Original GRU 775 / drive_backward_slow | 0.0 | 15.000 | 0.1576 | 0.0928 | 2.0200 | 0.1868 | 0.1810 |
| Original GRU 775 / drive_forward | 0.0 | 15.000 | 0.1630 | 0.0891 | 4.7947 | 0.1216 | 0.0866 |
| Original GRU 775 / drive_backward | 0.0 | 15.000 | 0.1604 | 0.1158 | 4.6727 | 0.1855 | 0.2068 |
| Original GRU 775 / turn_in_place | 0.0 | 15.000 | 0.0924 | 0.0803 | 0.3540 | 0.1147 | 0.0915 |
| Original GRU 775 / drive_and_turn | 0.0 | 15.000 | 0.1273 | 0.1052 | 1.7258 | 0.1035 | 0.0731 |
| Original GRU 775 / push_while_still | 0.0 | 15.000 | 0.0993 | 0.0704 | 0.5714 | 0.1241 | 0.1011 |
| Original GRU 775 / push_while_driving | 0.0 | 15.000 | 0.1670 | 0.1020 | 4.9329 | 0.1418 | 0.0921 |
| Original GRU 775 / payload_while_still | 0.0 | 15.000 | 0.1080 | 0.0816 | 0.6987 | 0.1306 | 0.1168 |
| Original GRU 775 / payload_while_driving | 0.0 | 15.000 | 0.2050 | 0.1084 | 5.7399 | 0.1619 | 0.1208 |
| First adaptation 874 / station_keeping | 0.0 | 15.000 | 0.0439 | 0.0509 | 0.0769 | 0.0592 | 0.0536 |
| First adaptation 874 / drive_forward_slow | 0.0 | 15.000 | 0.1256 | 0.0953 | 2.1746 | 0.0845 | 0.0648 |
| First adaptation 874 / drive_backward_slow | 0.0 | 15.000 | 0.1655 | 0.0989 | 3.0902 | 0.1119 | 0.1200 |
| First adaptation 874 / drive_forward | 0.0 | 15.000 | 0.1192 | 0.0999 | 4.1670 | 0.0793 | 0.0568 |
| First adaptation 874 / drive_backward | 0.0 | 15.000 | 0.1794 | 0.1676 | 5.5199 | 0.2476 | 0.3275 |
| First adaptation 874 / turn_in_place | 0.0 | 15.000 | 0.0742 | 0.0847 | 0.2221 | 0.0561 | 0.0534 |
| First adaptation 874 / drive_and_turn | 0.0 | 15.000 | 0.0869 | 0.1247 | 1.3881 | 0.0596 | 0.0389 |
| First adaptation 874 / push_while_still | 0.0 | 15.000 | 0.0660 | 0.0661 | 0.0940 | 0.0733 | 0.0620 |
| First adaptation 874 / push_while_driving | 0.0 | 15.000 | 0.1308 | 0.0970 | 4.2704 | 0.0813 | 0.0517 |
| First adaptation 874 / payload_while_still | 0.0 | 15.000 | 0.0667 | 0.0706 | 0.2730 | 0.0809 | 0.0817 |
| First adaptation 874 / payload_while_driving | 0.0 | 15.000 | 0.1466 | 0.1015 | 4.5272 | 0.0787 | 0.0534 |
| Failed fresh pilot 1150 / station_keeping | 0.0 | 15.000 | 0.0532 | 0.0473 | 0.0587 | 0.1340 | 0.2268 |
| Failed fresh pilot 1150 / drive_forward_slow | 0.0 | 15.000 | 0.0969 | 0.0688 | 2.0129 | 0.1624 | 0.2608 |
| Failed fresh pilot 1150 / drive_backward_slow | 0.0 | 15.000 | 0.0946 | 0.0645 | 1.8973 | 0.1569 | 0.2374 |
| Failed fresh pilot 1150 / drive_forward | 0.0 | 15.000 | 0.0808 | 0.0677 | 4.6530 | 0.1294 | 0.1971 |
| Failed fresh pilot 1150 / drive_backward | 0.0 | 15.000 | 0.0785 | 0.0636 | 4.0636 | 0.1298 | 0.1682 |
| Failed fresh pilot 1150 / turn_in_place | 0.0 | 15.000 | 0.0640 | 0.0688 | 0.1873 | 0.1229 | 0.2316 |
| Failed fresh pilot 1150 / drive_and_turn | 0.0 | 15.000 | 0.0711 | 0.0936 | 1.4421 | 0.1378 | 0.2503 |
| Failed fresh pilot 1150 / push_while_still | 0.0 | 15.000 | 0.0684 | 0.0501 | 0.0706 | 0.1354 | 0.2339 |
| Failed fresh pilot 1150 / push_while_driving | 0.0 | 15.000 | 0.0973 | 0.0727 | 4.6165 | 0.1538 | 0.2393 |
| Failed fresh pilot 1150 / payload_while_still | 0.0 | 15.000 | 0.0757 | 0.0514 | 0.2424 | 0.1340 | 0.2383 |
| Failed fresh pilot 1150 / payload_while_driving | 0.0 | 15.000 | 0.1074 | 0.0722 | 5.2218 | 0.1472 | 0.2349 |
| Adaptation final / station_keeping | 0.0 | 15.000 | 0.0459 | 0.0510 | 0.0844 | 0.1361 | 0.2346 |
| Adaptation final / drive_forward_slow | 0.0 | 15.000 | 0.0744 | 0.0719 | 1.7513 | 0.1508 | 0.2433 |
| Adaptation final / drive_backward_slow | 0.0 | 15.000 | 0.0992 | 0.0685 | 2.3286 | 0.1513 | 0.2505 |
| Adaptation final / drive_forward | 0.0 | 15.000 | 0.0663 | 0.0734 | 4.1779 | 0.1376 | 0.2211 |
| Adaptation final / drive_backward | 0.0 | 15.000 | 0.0632 | 0.0652 | 4.4002 | 0.1159 | 0.1739 |
| Adaptation final / turn_in_place | 0.0 | 15.000 | 0.0674 | 0.0692 | 0.2194 | 0.1439 | 0.2868 |
| Adaptation final / drive_and_turn | 0.0 | 15.000 | 0.0586 | 0.0909 | 1.3087 | 0.1345 | 0.2525 |
| Adaptation final / push_while_still | 0.0 | 15.000 | 0.0624 | 0.0501 | 0.1072 | 0.1167 | 0.1879 |
| Adaptation final / push_while_driving | 0.0 | 15.000 | 0.0809 | 0.0736 | 4.2735 | 0.1453 | 0.2282 |
| Adaptation final / payload_while_still | 0.0 | 15.000 | 0.0707 | 0.0556 | 0.2178 | 0.1394 | 0.2453 |
| Adaptation final / payload_while_driving | 0.0 | 15.000 | 0.0866 | 0.0745 | 4.6457 | 0.1386 | 0.2127 |

### Test — base seed 7001

| Model | Falls / 704 | Mean survival (s) | Minimum sample coverage (%) | Gate failures |
|---|---:|---:|---:|---|
| Original GRU 775 | 0 | 15.000 | 100.0 | station_keeping: world_drift_m 0.572 > 0.500; drive_forward_slow: rms_vel_err_mps 0.193 > 0.120; drive_backward_slow: rms_vel_err_mps 0.160 > 0.120; push_while_still: world_drift_m 0.599 > 0.500; payload_while_still: world_drift_m 0.686 > 0.500 |
| First adaptation 874 | 0 | 15.000 | 100.0 | drive_forward_slow: rms_vel_err_mps 0.135 > 0.120; drive_backward_slow: rms_vel_err_mps 0.174 > 0.120 |
| Failed fresh pilot 1150 | 0 | 15.000 | 100.0 | None |
| Adaptation final | 0 | 15.000 | 100.0 | None |

No invalid-state or physics-broken terminations occurred in this split. Coverage is the smallest scored-sample fraction across scenarios, after the common settling exclusion.

| Model / scenario | Falls (%) | Survival (s) | Velocity error RMS (m/s) | Yaw-rate error RMS (rad/s) | Final world drift (m) | Effective current RMS (A) | Command change (A/tick) |
|---|---:|---:|---:|---:|---:|---:|---:|
| Original GRU 775 / station_keeping | 0.0 | 15.000 | 0.0890 | 0.0655 | 0.5716 | 0.1188 | 0.1023 |
| Original GRU 775 / drive_forward_slow | 0.0 | 15.000 | 0.1933 | 0.1082 | 2.9002 | 0.1574 | 0.1099 |
| Original GRU 775 / drive_backward_slow | 0.0 | 15.000 | 0.1600 | 0.0860 | 2.2581 | 0.1705 | 0.1566 |
| Original GRU 775 / drive_forward | 0.0 | 15.000 | 0.1720 | 0.0922 | 4.8358 | 0.1419 | 0.0929 |
| Original GRU 775 / drive_backward | 0.0 | 15.000 | 0.1740 | 0.1281 | 4.8550 | 0.2043 | 0.2481 |
| Original GRU 775 / turn_in_place | 0.0 | 15.000 | 0.0980 | 0.0837 | 0.3465 | 0.1170 | 0.0943 |
| Original GRU 775 / drive_and_turn | 0.0 | 15.000 | 0.1404 | 0.1109 | 1.7747 | 0.1187 | 0.0802 |
| Original GRU 775 / push_while_still | 0.0 | 15.000 | 0.1089 | 0.0768 | 0.5987 | 0.1379 | 0.1128 |
| Original GRU 775 / push_while_driving | 0.0 | 15.000 | 0.1847 | 0.1026 | 4.9665 | 0.1416 | 0.0980 |
| Original GRU 775 / payload_while_still | 0.0 | 15.000 | 0.1105 | 0.0803 | 0.6859 | 0.1475 | 0.1320 |
| Original GRU 775 / payload_while_driving | 0.0 | 15.000 | 0.2065 | 0.1011 | 5.5163 | 0.1570 | 0.1027 |
| First adaptation 874 / station_keeping | 0.0 | 15.000 | 0.0460 | 0.0547 | 0.0656 | 0.0626 | 0.0591 |
| First adaptation 874 / drive_forward_slow | 0.0 | 15.000 | 0.1351 | 0.0918 | 2.1485 | 0.0857 | 0.0637 |
| First adaptation 874 / drive_backward_slow | 0.0 | 15.000 | 0.1744 | 0.1167 | 3.0870 | 0.1573 | 0.1878 |
| First adaptation 874 / drive_forward | 0.0 | 15.000 | 0.1411 | 0.0926 | 4.4021 | 0.0784 | 0.0555 |
| First adaptation 874 / drive_backward | 0.0 | 15.000 | 0.1875 | 0.1598 | 5.5312 | 0.2263 | 0.2961 |
| First adaptation 874 / turn_in_place | 0.0 | 15.000 | 0.0796 | 0.0917 | 0.2244 | 0.0651 | 0.0537 |
| First adaptation 874 / drive_and_turn | 0.0 | 15.000 | 0.0937 | 0.1358 | 1.4330 | 0.0695 | 0.0422 |
| First adaptation 874 / push_while_still | 0.0 | 15.000 | 0.0669 | 0.0700 | 0.0959 | 0.0770 | 0.0660 |
| First adaptation 874 / push_while_driving | 0.0 | 15.000 | 0.1508 | 0.1006 | 4.3829 | 0.0885 | 0.0615 |
| First adaptation 874 / payload_while_still | 0.0 | 15.000 | 0.0726 | 0.0739 | 0.3050 | 0.0828 | 0.0812 |
| First adaptation 874 / payload_while_driving | 0.0 | 15.000 | 0.1746 | 0.1035 | 4.8106 | 0.0976 | 0.0701 |
| Failed fresh pilot 1150 / station_keeping | 0.0 | 15.000 | 0.0520 | 0.0459 | 0.0610 | 0.1259 | 0.2222 |
| Failed fresh pilot 1150 / drive_forward_slow | 0.0 | 15.000 | 0.0886 | 0.0689 | 2.0040 | 0.1355 | 0.2063 |
| Failed fresh pilot 1150 / drive_backward_slow | 0.0 | 15.000 | 0.0930 | 0.0616 | 1.9849 | 0.1538 | 0.2233 |
| Failed fresh pilot 1150 / drive_forward | 0.0 | 15.000 | 0.0890 | 0.0694 | 4.6902 | 0.1463 | 0.2259 |
| Failed fresh pilot 1150 / drive_backward | 0.0 | 15.000 | 0.0810 | 0.0680 | 4.0187 | 0.1360 | 0.1749 |
| Failed fresh pilot 1150 / turn_in_place | 0.0 | 15.000 | 0.0697 | 0.0795 | 0.1661 | 0.1628 | 0.2989 |
| Failed fresh pilot 1150 / drive_and_turn | 0.0 | 15.000 | 0.0740 | 0.0979 | 1.4983 | 0.1351 | 0.2372 |
| Failed fresh pilot 1150 / push_while_still | 0.0 | 15.000 | 0.0714 | 0.0544 | 0.0700 | 0.1476 | 0.2593 |
| Failed fresh pilot 1150 / push_while_driving | 0.0 | 15.000 | 0.0952 | 0.0724 | 4.6506 | 0.1496 | 0.2340 |
| Failed fresh pilot 1150 / payload_while_still | 0.0 | 15.000 | 0.0666 | 0.0521 | 0.2128 | 0.1295 | 0.2256 |
| Failed fresh pilot 1150 / payload_while_driving | 0.0 | 15.000 | 0.1042 | 0.0726 | 5.1087 | 0.1500 | 0.2465 |
| Adaptation final / station_keeping | 0.0 | 15.000 | 0.0498 | 0.0475 | 0.0746 | 0.1195 | 0.1987 |
| Adaptation final / drive_forward_slow | 0.0 | 15.000 | 0.0765 | 0.0772 | 1.7345 | 0.1446 | 0.2278 |
| Adaptation final / drive_backward_slow | 0.0 | 15.000 | 0.1016 | 0.0696 | 2.3411 | 0.1428 | 0.2251 |
| Adaptation final / drive_forward | 0.0 | 15.000 | 0.0704 | 0.0739 | 4.3349 | 0.1576 | 0.2520 |
| Adaptation final / drive_backward | 0.0 | 15.000 | 0.0695 | 0.0687 | 4.3824 | 0.1235 | 0.1765 |
| Adaptation final / turn_in_place | 0.0 | 15.000 | 0.0649 | 0.0698 | 0.2176 | 0.1319 | 0.2543 |
| Adaptation final / drive_and_turn | 0.0 | 15.000 | 0.0594 | 0.0961 | 1.3195 | 0.1280 | 0.2234 |
| Adaptation final / push_while_still | 0.0 | 15.000 | 0.0612 | 0.0490 | 0.1038 | 0.1084 | 0.1700 |
| Adaptation final / push_while_driving | 0.0 | 15.000 | 0.0781 | 0.0734 | 4.2055 | 0.1377 | 0.1981 |
| Adaptation final / payload_while_still | 0.0 | 15.000 | 0.0712 | 0.0590 | 0.2763 | 0.1410 | 0.2451 |
| Adaptation final / payload_while_driving | 0.0 | 15.000 | 0.0848 | 0.0738 | 4.8329 | 0.1310 | 0.1850 |

### Predeclared standing-current screen

Required: candidate mean absolute command change no greater than 75% of the parent pilot on both splits, in addition to all stage-5 gates.

- validation: parent 0.2268 A/tick; candidate 0.2346 A/tick; ratio 1.034; pass=False.
- test: parent 0.2222 A/tick; candidate 0.1987 A/tick; ratio 0.894; pass=False.

## Physical interpretation

The supplied fresh-C capture contains one unforced standing failure, 5.336 s after rearm, before testbench start. Its network outputs match the expected graph to 7.2e-7 A. The observation-timing mismatch is a confirmed implementation issue; its complete causal contribution to the fall remains unproven. No new physical trial was performed by this investigation.

The unequal-terrain results from the first harness version are excluded and retained under `logs/regression_diagnosis_2026-10-07/invalid_unbalanced_terrain`. All tables above use the repaired, explicitly balanced terrain layout.

The original hardware archive remains usable as implementation-bound feasibility evidence with disclosed limitations. See PAPER_PATH.md. No failed seed, failed capture or unsuccessful adaptation is removed. Warm-start adaptation is not an independent training replicate.
