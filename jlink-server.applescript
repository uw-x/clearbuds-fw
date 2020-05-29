tell application "iTerm2"
    set newWindow to (create window with default profile)
    tell current session of newWindow
        write text "JLinkExe -device NRF52 -speed 4000 -if SWD -AutoConnect 1"
    end tell
end tell
