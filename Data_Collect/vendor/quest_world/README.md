# Piper Quest World input

独立 Android NativeActivity/OpenXR 输入程序，仅输出右手柄 LOCAL grip/aim 位姿和按钮，不连接机器人。旧相对坐标 APK 保持独立。

依赖 Meta OpenXR SDK v85 (`bbed2f20e38a5df7113630771c83cb8279e4fc26`) 的 SampleXrFramework 和 Khronos OpenXR loader 1.1.53。Meta SDK 在构建时获取，按其 [SDK 许可证](https://github.com/meta-quest/Meta-OpenXR-SDK/blob/v85/LICENSE.txt) 使用；Gradle wrapper 来自 [oculus_reader](https://github.com/jborbik/oculus_reader/tree/9689484d319c4798e54d59509b192436647b7427/app/gradle)，采用 Gradle 的 Apache 2.0 许可证。借鉴其 Gradle/SDK 集成方法；原生采集程序单独实现，未复用其控制器 UI 或 beta 输入实现。

构建依赖：Java 17/21、Android SDK platform-tools、platforms;android-32、build-tools;33.0.1、NDK 27.0.12077973、CMake 3.22.1。Gradle 8.5，AGP 8.1.4。构建目录和 APK 均为生成文件，不提交。

```bash
cd /home/yanyu/Documents/Piper-main/Data_Collect/vendor/quest_world/app
export ANDROID_HOME=/home/yanyu/Documents/Piper-main/Data_Collect/.runtime/android/sdk
./gradlew --no-daemon assembleDebug
```

已有 SDK 源码时可追加 `-PmetaSdkDir=/home/yanyu/Documents/Piper-main/Data_Collect/.runtime/meta-openxr-sdk`，避免重复下载。输出为 `app/build/outputs/apk/debug/PiperQuestWorld-debug.apk`。

安装和只读核验：

```bash
cd /home/yanyu/Documents/Piper-main/Data_Collect
source env.sh capture
python -m data_collect.cli.quest_world --install --check --duration 10
```

数据通过 logcat 标签 `PiperQuestWorld` 发送 JSON，协议 `piper.quest.local.v1`。位置单位米，四元数顺序 xyzw；`device_monotonic_ns` 使用 Android CLOCK_BOOTTIME，`xr_sample_time_ns` 是 OpenXR 预测显示采样时间，不能直接当作工作站时间。接收端用启动时 BOOTTIME 采样和时钟差检测旧包，保留源序号，不将重复轮询包装成新追踪帧。

SDK v85 的 `XrApp.cpp` 假定 Android onResume/onPause 先于 OpenXR READY/STOPPING；本机 Quest 3 唤醒/前台切换中实际出现相反事件顺序并导致 SIGABRT。CMake 仅在构建副本中去掉这两条顺序断言，保留 `xrBeginSession`/`xrEndSession`、SessionActive 断言及其余检查，不改共享 SDK 源码。依据 [OpenXR 会话状态](https://registry.khronos.org/OpenXR/specs/1.1/man/html/XrSessionState.html)，READY/STOPPING 应正常开始/结束会话，不能假定 Android 生命周期的回调顺序。
