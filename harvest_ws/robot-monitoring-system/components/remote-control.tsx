"use client"

import {useState, useCallback, useEffect} from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import { Button } from "@/components/ui/button"
import { Slider } from "@/components/ui/slider"
import { Switch } from "@/components/ui/switch"
import { Label } from "@/components/ui/label"
import {
  Gamepad2,
  ArrowUp,
  ArrowDown,
  ArrowLeft,
  ArrowRight,
  RotateCcw,
  RotateCw,
  Square,
  Hand,
  Zap,
  Home,
  Power,
  AlertTriangle,
} from "lucide-react"

interface ControlState {
  isEnabled: boolean
  isEmergencyStop: boolean
  speed: number
  armEnabled: boolean
  gripperOpen: boolean
}

export function RemoteControl() {
  const [state, setState] = useState<ControlState>({
    isEnabled: false,
    isEmergencyStop: false,
    speed: 50,
    armEnabled: false,
    gripperOpen: true,
  })
  const [activeKeys, setActiveKeys] = useState<Set<string>>(new Set())
  const [lastCommand, setLastCommand] = useState<string>("Idle")

  const handleCommand = useCallback(
    (command: string, direction: string) => {
      if (!state.isEnabled || state.isEmergencyStop) return
      setLastCommand(`${command}: ${direction}`)
      setTimeout(() => setLastCommand("Idle"), 500)
    },
    [state.isEnabled, state.isEmergencyStop]
  )

  const handleKeyPress = (key: string, pressed: boolean) => {
    setActiveKeys((prev) => {
      const newSet = new Set(prev)
      if (pressed) {
        newSet.add(key)
      } else {
        newSet.delete(key)
      }
      return newSet
    })
  }
  const sendMove = async (x = 0, y = 0, z = 0, r = 20, roughly = 0, lr = 1) => {
    const params = new URLSearchParams({
      x: String(x),
      y: String(y),
      z: String(z),
      r: String(r),
      roughly: String(roughly),
      lr: String(lr),
    })

    await fetch(`http://172.23.248.32:5002/move/custom?${params}`)
  }

  useEffect(() => {
    if (!activeKeys.size) {
   //   sendMove(0, 0, 0)
      return
    }

    let x = 0
    let y = 0
    let z = 0

    activeKeys.forEach((k) => {
      console.log(k)
      switch (k) {
        case "forward":
          x += 10
          break
        case "backward":
          x -= 10
          break
        case "left":
          y += 10
          break
        case "right":
          y -= 10
          break
        case "up":
          z += 10
          break
        case "down":
          z -= 10
          break
      }
    })

    sendMove(x, y, z)
  }, [activeKeys])


  const emergencyStop = () => {
    setState((prev) => ({
      ...prev,
      isEmergencyStop: !prev.isEmergencyStop,
      isEnabled: false,
    }))
    setLastCommand(state.isEmergencyStop ? "Emergency Stop Released" : "EMERGENCY STOP ACTIVATED")
  }

  return (
    <Card className="border-border bg-card">
      <CardHeader className="flex flex-row items-center justify-between py-3 px-4 border-b border-border">
        <div className="flex items-center gap-3">
          <Gamepad2 className="h-4 w-4 text-muted-foreground" />
          <CardTitle className="text-sm font-medium">Remote Control</CardTitle>
          <Badge
            variant={state.isEnabled ? "default" : "secondary"}
            className={
              state.isEmergencyStop
                ? "bg-destructive/20 text-destructive border-destructive/30"
                : state.isEnabled
                  ? "bg-success/20 text-success border-success/30"
                  : ""
            }
          >
            {state.isEmergencyStop
              ? "E-STOP"
              : state.isEnabled
                ? "Active"
                : "Disabled"}
          </Badge>
        </div>
        <div className="flex items-center gap-2">
          <Switch
            checked={state.isEnabled}
            onCheckedChange={(checked) =>
              setState((prev) => ({
                ...prev,
                isEnabled: checked,
                isEmergencyStop: false,
              }))
            }
            disabled={state.isEmergencyStop}
          />
          <Label className="text-xs text-muted-foreground">Enable</Label>
        </div>
      </CardHeader>
      <CardContent className="p-4 space-y-4">
        {/* Emergency Stop */}
        <Button
          variant={state.isEmergencyStop ? "destructive" : "outline"}
          className={`w-full h-12 text-sm font-bold ${state.isEmergencyStop ? "" : "border-destructive text-destructive hover:bg-destructive hover:text-destructive-foreground"}`}
          onClick={emergencyStop}
        >
          <AlertTriangle className="h-5 w-5 mr-2" />
          {state.isEmergencyStop ? "RELEASE E-STOP" : "EMERGENCY STOP"}
        </Button>

        {/* Movement Controls */}
        <div className="space-y-2">
          <Label className="text-xs text-muted-foreground">
            Movement Control
          </Label>
          <div className="grid grid-cols-3 gap-2 max-w-[180px] mx-auto">
            <div />
            <Button
              variant="secondary"
              size="icon"
              className={`h-12 w-full ${activeKeys.has("forward") ? "bg-primary text-primary-foreground" : ""}`}
              disabled={!state.isEnabled || state.isEmergencyStop}
              onMouseDown={() => {
                handleKeyPress("forward", true)
                handleCommand("Move", "Forward")
              }}
              onMouseUp={() => handleKeyPress("forward", false)}
              onMouseLeave={() => handleKeyPress("forward", false)}
            >
              <ArrowUp className="h-5 w-5" />
            </Button>
            <div />

            <Button
              variant="secondary"
              size="icon"
              className={`h-12 w-full ${activeKeys.has("left") ? "bg-primary text-primary-foreground" : ""}`}
              disabled={!state.isEnabled || state.isEmergencyStop}
              onMouseDown={() => {
                handleKeyPress("left", true)
                handleCommand("Move", "Left")
              }}
              onMouseUp={() => handleKeyPress("left", false)}
              onMouseLeave={() => handleKeyPress("left", false)}
            >
              <ArrowLeft className="h-5 w-5" />
            </Button>
            <Button
              variant="secondary"
              size="icon"
              className={`h-12 w-full ${activeKeys.has("stop") ? "bg-primary text-primary-foreground" : ""}`}
              disabled={!state.isEnabled || state.isEmergencyStop}
              onMouseDown={() => {
                handleKeyPress("stop", true)
                handleCommand("Stop", "All")
              }}
              onMouseUp={() => handleKeyPress("stop", false)}
            >
              <Square className="h-4 w-4" />
            </Button>
            <Button
              variant="secondary"
              size="icon"
              className={`h-12 w-full ${activeKeys.has("right") ? "bg-primary text-primary-foreground" : ""}`}
              disabled={!state.isEnabled || state.isEmergencyStop}
              onMouseDown={() => {
                handleKeyPress("right", true)
                handleCommand("Move", "Right")
              }}
              onMouseUp={() => handleKeyPress("right", false)}
              onMouseLeave={() => handleKeyPress("right", false)}
            >
              <ArrowRight className="h-5 w-5" />
            </Button>

            <div />
            <Button
              variant="secondary"
              size="icon"
              className={`h-12 w-full ${activeKeys.has("backward") ? "bg-primary text-primary-foreground" : ""}`}
              disabled={!state.isEnabled || state.isEmergencyStop}
              onMouseDown={() => {
                handleKeyPress("backward", true)
                handleCommand("Move", "Backward")
              }}
              onMouseUp={() => handleKeyPress("backward", false)}
              onMouseLeave={() => handleKeyPress("backward", false)}
            >
              <ArrowDown className="h-5 w-5" />
            </Button>
            <div />
          </div>
        </div>

        {/* Rotation Controls */}
        <div className="flex justify-center gap-4">
          <Button
            variant="secondary"
            size="sm"
            className={`flex-1 ${activeKeys.has("rotateLeft") ? "bg-primary text-primary-foreground" : ""}`}
            disabled={!state.isEnabled || state.isEmergencyStop}
            onMouseDown={() => {
              handleKeyPress("rotateLeft", true)
              handleCommand("Rotate", "CCW")
            }}
            onMouseUp={() => handleKeyPress("rotateLeft", false)}
            onMouseLeave={() => handleKeyPress("rotateLeft", false)}
          >
            <RotateCcw className="h-4 w-4 mr-2" />
            Rotate CCW
          </Button>
          <Button
            variant="secondary"
            size="sm"
            className={`flex-1 ${activeKeys.has("rotateRight") ? "bg-primary text-primary-foreground" : ""}`}
            disabled={!state.isEnabled || state.isEmergencyStop}
            onMouseDown={() => {
              handleKeyPress("rotateRight", true)
              handleCommand("Rotate", "CW")
            }}
            onMouseUp={() => handleKeyPress("rotateRight", false)}
            onMouseLeave={() => handleKeyPress("rotateRight", false)}
          >
            <RotateCw className="h-4 w-4 mr-2" />
            Rotate CW
          </Button>
        </div>

        {/* Speed Control */}
        <div className="space-y-2">
          <div className="flex justify-between items-center">
            <Label className="text-xs text-muted-foreground">
              Speed Control
            </Label>
            <span className="text-xs font-mono text-foreground">
              {state.speed}%
            </span>
          </div>
          <Slider
            value={[state.speed]}
            onValueChange={([value]) =>
              setState((prev) => ({ ...prev, speed: value }))
            }
            max={100}
            step={5}
            disabled={!state.isEnabled || state.isEmergencyStop}
            className="w-full"
          />
        </div>

        {/* Arm & Gripper Controls */}
        <div className="space-y-3 pt-2 border-t border-border">
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-2">
              <Zap className="h-4 w-4 text-muted-foreground" />
              <Label className="text-sm">Arm Control</Label>
            </div>
            <Switch
              checked={state.armEnabled}
              onCheckedChange={(checked) =>
                setState((prev) => ({ ...prev, armEnabled: checked }))
              }
              disabled={!state.isEnabled || state.isEmergencyStop}
            />
          </div>
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-2">
              <Hand className="h-4 w-4 text-muted-foreground" />
              <Label className="text-sm">Gripper</Label>
            </div>
            <Button
              variant={state.gripperOpen ? "outline" : "secondary"}
              size="sm"
              onClick={() =>
                setState((prev) => ({
                  ...prev,
                  gripperOpen: !prev.gripperOpen,
                }))
              }
              disabled={
                !state.isEnabled || state.isEmergencyStop || !state.armEnabled
              }
            >
              {state.gripperOpen ? "Open" : "Closed"}
            </Button>
          </div>
        </div>

        {/* Quick Actions */}
        <div className="grid grid-cols-2 gap-2 pt-2 border-t border-border">
          <Button
            variant="outline"
            size="sm"
            disabled={!state.isEnabled || state.isEmergencyStop}
            onClick={() => handleCommand("Action", "Home Position")}
          >
            <Home className="h-4 w-4 mr-2" />
            Home
          </Button>
          <Button
            variant="outline"
            size="sm"
            disabled={state.isEmergencyStop}
            onClick={() =>
              setState((prev) => ({ ...prev, isEnabled: !prev.isEnabled }))
            }
          >
            <Power className="h-4 w-4 mr-2" />
            {state.isEnabled ? "Disable" : "Enable"}
          </Button>
        </div>

        {/* Status */}
        <div className="flex items-center justify-between px-3 py-2 bg-secondary rounded-md">
          <span className="text-xs text-muted-foreground">Last Command:</span>
          <span className="text-xs font-mono text-foreground">
            {lastCommand}
          </span>
        </div>
      </CardContent>
    </Card>
  )
}
