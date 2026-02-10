"use client"

import { useState, useEffect } from "react"
import { Badge } from "@/components/ui/badge"
import { Button } from "@/components/ui/button"
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuLabel,
  DropdownMenuSeparator,
  DropdownMenuTrigger,
} from "@/components/ui/dropdown-menu"
import {
  Bot,
  Settings,
  Bell,
  ChevronDown,
  Circle,
  RefreshCw,
  Moon,
  Sun,
  User,
} from "lucide-react"

interface Robot {
  id: string
  name: string
  status: "online" | "offline" | "maintenance"
}

const robots: Robot[] = [
  { id: "robot-001", name: "Assembly Bot Alpha", status: "online" },
  { id: "robot-002", name: "Transport Unit Beta", status: "online" },
  { id: "robot-003", name: "Inspection Drone Gamma", status: "maintenance" },
  { id: "robot-004", name: "Welding Arm Delta", status: "offline" },
]

export function DashboardHeader() {
  const [selectedRobot, setSelectedRobot] = useState(robots[0])
  const [notifications, setNotifications] = useState(3)
  const [isDark, setIsDark] = useState(false)
  const [currentTime, setCurrentTime] = useState(new Date())

  useEffect(() => {
    const interval = setInterval(() => {
      setCurrentTime(new Date())
    }, 1000)
    return () => clearInterval(interval)
  }, [])

  const getStatusColor = (status: string) => {
    switch (status) {
      case "online":
        return "bg-green-500/20 text-green-400 border-green-500/30"
      case "maintenance":
        return "bg-yellow-500/20 text-yellow-400 border-yellow-500/30"
      case "offline":
        return "bg-red-500/20 text-red-400 border-red-500/30"
      default:
        return ""
    }
  }

  const getStatusDot = (status: string) => {
    switch (status) {
      case "online":
        return "fill-green-400"
      case "maintenance":
        return "fill-yellow-400"
      case "offline":
        return "fill-red-400"
      default:
        return ""
    }
  }

  return (
      // Hardcoded dark colors for the header to maintain the black appearance on white body
      <header className="sticky top-0 z-50 border-b border-white/10 bg-[#09090b] text-white backdrop-blur supports-[backdrop-filter]:bg-[#09090b]/90">
        <div className="flex h-14 items-center justify-between px-4 lg:px-6">
          {/* Logo & Robot Selector */}
          <div className="flex items-center gap-4">
            <div className="flex items-center gap-2">
              <div className="flex h-8 w-8 items-center justify-center rounded-md bg-primary text-primary-foreground">
                <Bot className="h-5 w-5" />
              </div>
              <span className="font-semibold text-white hidden sm:inline">
              RobotOS
            </span>
            </div>

            <div className="hidden sm:block h-6 w-px bg-white/10" />

            <DropdownMenu>
              <DropdownMenuTrigger asChild>
                <Button
                    variant="ghost"
                    className="flex items-center gap-2 h-9 px-3 text-white hover:bg-white/10 hover:text-white data-[state=open]:bg-white/10"
                >
                  <Circle
                      className={`h-2 w-2 ${getStatusDot(selectedRobot.status)}`}
                  />
                  <span className="text-sm font-medium hidden md:inline">
                  {selectedRobot.name}
                </span>
                  <span className="text-sm font-medium md:hidden">
                  {selectedRobot.id}
                </span>
                  <ChevronDown className="h-4 w-4 text-white/70" />
                </Button>
              </DropdownMenuTrigger>
              <DropdownMenuContent align="start" className="w-56">
                <DropdownMenuLabel>Select Robot</DropdownMenuLabel>
                <DropdownMenuSeparator />
                {robots.map((robot) => (
                    <DropdownMenuItem
                        key={robot.id}
                        onClick={() => setSelectedRobot(robot)}
                        className="flex items-center justify-between cursor-pointer"
                    >
                      <div className="flex items-center gap-2">
                        <Circle className={`h-2 w-2 ${getStatusDot(robot.status)}`} />
                        <span>{robot.name}</span>
                      </div>
                      {/* Reverting to default badge for dropdown content which adapts to theme */}
                      <Badge variant="outline" className="text-xs">
                        {robot.status}
                      </Badge>
                    </DropdownMenuItem>
                ))}
              </DropdownMenuContent>
            </DropdownMenu>
          </div>

          {/* Center - Status Badge */}
          <div className="hidden lg:flex items-center gap-2">
            <Badge
                variant="outline"
                className={getStatusColor(selectedRobot.status)}
            >
              <Circle
                  className={`h-2 w-2 mr-1 ${getStatusDot(selectedRobot.status)} ${selectedRobot.status === "online" ? "animate-pulse" : ""}`}
              />
              {selectedRobot.status.toUpperCase()}
            </Badge>
            <span className="text-xs text-white/50 font-mono">
            {currentTime.toLocaleTimeString()}
          </span>
          </div>

          {/* Right Side Actions */}
          <div className="flex items-center gap-2">
            <Button variant="ghost" size="icon" className="h-9 w-9 text-white hover:bg-white/10 hover:text-white">
              <RefreshCw className="h-4 w-4" />
              <span className="sr-only">Refresh</span>
            </Button>

            <Button
                variant="ghost"
                size="icon"
                className="h-9 w-9 relative text-white hover:bg-white/10 hover:text-white"
                onClick={() => setNotifications(0)}
            >
              <Bell className="h-4 w-4" />
              {notifications > 0 && (
                  <span className="absolute -top-0.5 -right-0.5 h-4 w-4 rounded-full bg-red-600 text-[10px] font-medium text-white flex items-center justify-center">
                {notifications}
              </span>
              )}
              <span className="sr-only">Notifications</span>
            </Button>

            <Button
                variant="ghost"
                size="icon"
                className="h-9 w-9 text-white hover:bg-white/10 hover:text-white"
                onClick={() => setIsDark(!isDark)}
            >
              {isDark ? (
                  <Sun className="h-4 w-4" />
              ) : (
                  <Moon className="h-4 w-4" />
              )}
              <span className="sr-only">Toggle theme</span>
            </Button>

            <Button variant="ghost" size="icon" className="h-9 w-9 text-white hover:bg-white/10 hover:text-white">
              <Settings className="h-4 w-4" />
              <span className="sr-only">Settings</span>
            </Button>

            <div className="hidden sm:block h-6 w-px bg-white/10" />

            <Button variant="ghost" size="icon" className="h-9 w-9 text-white hover:bg-white/10 hover:text-white">
              <User className="h-4 w-4" />
              <span className="sr-only">User</span>
            </Button>
          </div>
        </div>
      </header>
  )
}