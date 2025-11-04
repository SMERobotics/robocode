# robocode

This repo contains the code used by Shawnee Mission East Robotics for the FTC DECODE 2025-2026 season.

### teams and maintainers

- [20181 (SME Lancers)](https://github.com/SMERobotics/robocode/tree/master/TeamCode2026/src/main/java/com/mech/ftc) the oldest sibling
  - @SM-mech (Jake Winfield)
- [26855 (SME Cavalry)](https://github.com/SMERobotics/robocode/tree/master/TeamCode2028/src/main/java/com/technodot/ftc) the middle child 🥀
  - @TechDudie (TechnoDot)
- [21692 (SME Knights)](https://github.com/SMERobotics/robocode/tree/master/TeamCode2029/src/main/java/com/n0tasha4k/ftc) the youngest sibling
  - @n0tasha4k-glitch (???)

### setup

Java 17 and Android Studio Otter (2025.2.1) recommended!

Create a Personal Access Token for Github and configure Android Studio to use the token for all git operations. Clone the repository.

### team

Do not decide to use Java "just because you can." Ensuring that everything works **when you need it** and **at the same time** is a massive pain. You should be very familiar with Java, using an IDE and API/SDK, and Git or other VCS. If you decide to use Java:

1. Copy the `TeamCode` directory and rename it to `TeamCode{year}`. `{year}` should be what class the majority of the team members are in. If there are multiple teams with the same year using Java, the teams should be `TeamCode{year}A`, `TeamCode{year}B`, and so forth.
2. Rename the `com.firstinspires.ftc.teamcode` package to `com.{name}.ftc.{year}` (ex. `com.technodot.ftc.twentyfive`. Your name can be your username, your real name, whatever you prefer (all lowercase, no spaces, dashes, or underscores).  The year should be the current year, spelled out, all lowercase without any spaces.
3. In `settings.gradle`, add a new line with `include ':TeamCode{year}'`.
4. In `TeamCode{year}/build.gradle`, change the `namespace` variable to your package name.
5. You may need to rebuild Gradle/restart Android Studio for the changes to take effect.
6. To the left of the Run button at the top of Android Studio, change the module being built to `TeamCode{year}`.
