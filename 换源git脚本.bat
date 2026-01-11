@echo off
chcp 65001 >nul

for /f %%b in ('git branch -r ^| findstr origin/') do git checkout -B %%b %%b

echo ==============================
echo 当前 Git 远程地址：
echo ==============================
git remote -v
echo.

set /p NEW_URL=请输入新的 Git 仓库地址（完整 URL）：

if "%NEW_URL%"=="" (
    echo 未输入地址，操作取消。
    pause
    exit /b 1
)

echo.
echo 正在更新所有 remote 的地址（对所有分支生效）...
for /f %%r in ('git remote') do (
    echo 修改 remote: %%r
    git remote set-url %%r %NEW_URL%
)

echo.
echo ==============================
echo 修改完成（所有分支已自动使用新地址）：
echo ==============================
git remote -v
echo.

git push --mirror

git fetch --all --prune
git branch
git push origin --all --force
git push origin --tags --force

git checkout main
git branch

pause
