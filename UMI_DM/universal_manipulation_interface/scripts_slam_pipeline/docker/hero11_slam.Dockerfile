# Reproduce the local SLAM image used by the configured HERO11 pipeline.
FROM chicheng/orb_slam3:latest
RUN python3 -c "from pathlib import Path; p=Path('/ORB_SLAM3/src/TwoViewReconstruction.cc'); s=p.read_text(); old='T21 = Sophus::SE3f(vR[bestSolutionIdx], vt[bestSolutionIdx]);\n            vbTriangulated = bestTriangulated;'; new='T21 = Sophus::SE3f(vR[bestSolutionIdx], vt[bestSolutionIdx]);\n            vP3D = bestP3D;\n            vbTriangulated = bestTriangulated;'; assert s.count(old) == 1; p.write_text(s.replace(old,new))"
RUN cmake --build /ORB_SLAM3/build --target ORB_SLAM3 -j 2
