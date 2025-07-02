@echo off
echo 🔧 브랜치 문제 해결 및 AUTO 저장소 업로드
echo =========================================

echo 📋 현재 상태 확인...
git status

echo 🌿 main 브랜치로 변경...
git checkout -b main

echo 📤 AUTO 저장소에 업로드...
git push -u origin main

if %ERRORLEVEL%==0 (
    echo ✅ 성공! AUTO 저장소 업로드 완료!
    echo 🌐 https://github.com/ThugGyu/AUTO
) else (
    echo ❌ 여전히 문제가 있습니다.
    echo 💡 GitHub Desktop 또는 VS Code를 사용해보세요.
)

pause 