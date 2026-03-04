"use client"

import React from "react"

import { useState, useEffect } from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import { Progress } from "@/components/ui/progress"
import {
  Activity,
  Battery,
  Cpu,
  Thermometer,
  Wifi,
  HardDrive,
  Clock,
  Signal,
} from "lucide-react"

interface StatusMetric {
  label: string
  value: number
  unit: string
  icon: React.ReactNode
  status: "normal" | "warning" | "critical"
}

export function StatusPanel() {
  const [metrics, setMetrics] = useState<StatusMetric[]>([
    {
      label: "Battery",
      value: 87,
      unit: "%",
      icon: <Battery className="h-4 w-4" />,
      status: "normal",
    },
    {
      label: "CPU",
      value: 42,
      unit: "%",
      icon: <Cpu className="h-4 w-4" />,
      status: "normal",
    },
    {
      label: "Temperature",
      value: 38,
      unit: "°C",
      icon: <Thermometer className="h-4 w-4" />,
      status: "normal",
    },
    {
      label: "Signal",
      value: 92,
      unit: "%",
      icon: <Wifi className="h-4 w-4" />,
      status: "normal",
    },
    {
      label: "Memory",
      value: 64,
      unit: "%",
      icon: <HardDrive className="h-4 w-4" />,
      status: "normal",
    },
    {
      label: "Latency",
      value: 45,
      unit: "ms",
      icon: <Signal className="h-4 w-4" />,
      status: "normal",
    },
  ])

  const [uptime, setUptime] = useState(0)

  useEffect(() => {
    const interval = setInterval(() => {
      setMetrics((prev) =>
        prev.map((metric) => {
          let newValue = metric.value + (Math.random() * 6 - 3)

          // Keep values in reasonable ranges
          if (metric.label === "Battery") {
            newValue = Math.max(20, Math.min(100, newValue - 0.1))
          } else if (metric.label === "Temperature") {
            newValue = Math.max(30, Math.min(75, newValue))
          } else if (metric.label === "Latency") {
            newValue = Math.max(20, Math.min(200, newValue))
          } else {
            newValue = Math.max(0, Math.min(100, newValue))
          }

          // Determine status
          let status: "normal" | "warning" | "critical" = "normal"
          if (metric.label === "Battery" && newValue < 30) {
            status = newValue < 15 ? "critical" : "warning"
          } else if (metric.label === "Temperature" && newValue > 55) {
            status = newValue > 65 ? "critical" : "warning"
          } else if (metric.label === "CPU" && newValue > 80) {
            status = newValue > 90 ? "critical" : "warning"
          } else if (metric.label === "Memory" && newValue > 85) {
            status = newValue > 95 ? "critical" : "warning"
          } else if (metric.label === "Latency" && newValue > 100) {
            status = newValue > 150 ? "critical" : "warning"
          }

          return { ...metric, value: Math.round(newValue), status }
        })
      )
      setUptime((prev) => prev + 1)
    }, 2000)

    return () => clearInterval(interval)
  }, [])

  const formatUptime = (seconds: number) => {
    const hrs = Math.floor(seconds / 3600)
    const mins = Math.floor((seconds % 3600) / 60)
    const secs = seconds % 60
    return `${hrs.toString().padStart(2, "0")}:${mins.toString().padStart(2, "0")}:${secs.toString().padStart(2, "0")}`
  }

  const getStatusColor = (status: string) => {
    switch (status) {
      case "critical":
        return "text-destructive"
      case "warning":
        return "text-warning"
      default:
        return "text-success"
    }
  }

  const getProgressColor = (status: string) => {
    switch (status) {
      case "critical":
        return "[&>div]:bg-destructive"
      case "warning":
        return "[&>div]:bg-warning"
      default:
        return "[&>div]:bg-success"
    }
  }

  return (
    <Card className="border-border bg-card">
      <CardHeader className="flex flex-row items-center justify-between py-3 px-4 border-b border-border">
        <div className="flex items-center gap-3">
          <Activity className="h-4 w-4 text-muted-foreground" />
          <CardTitle className="text-sm font-medium">System Status</CardTitle>
        </div>
        <Badge variant="secondary" className="text-xs font-mono">
          <Clock className="h-3 w-3 mr-1" />
          {formatUptime(uptime)}
        </Badge>
      </CardHeader>
      <CardContent className="p-4">
        <div className="grid grid-cols-2 gap-4">
          {metrics.map((metric) => (
            <div key={metric.label} className="space-y-2">
              <div className="flex items-center justify-between">
                <div className="flex items-center gap-2">
                  <span className={getStatusColor(metric.status)}>
                    {metric.icon}
                  </span>
                  <span className="text-xs text-muted-foreground">
                    {metric.label}
                  </span>
                </div>
                <span
                  className={`text-sm font-mono ${getStatusColor(metric.status)}`}
                >
                  {metric.value}
                  {metric.unit}
                </span>
              </div>
              <Progress
                value={
                  metric.label === "Latency"
                    ? Math.min((metric.value / 200) * 100, 100)
                    : metric.label === "Temperature"
                      ? (metric.value / 80) * 100
                      : metric.value
                }
                className={`h-1.5 ${getProgressColor(metric.status)}`}
              />
            </div>
          ))}
        </div>

        {/* Connection Status */}
        <div className="mt-4 pt-4 border-t border-border">
          <div className="flex items-center justify-between">
            <span className="text-xs text-muted-foreground">
              Connection Status
            </span>
            <div className="flex items-center gap-2">
              <span className="relative flex h-2 w-2">
                <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-success opacity-75"></span>
                <span className="relative inline-flex rounded-full h-2 w-2 bg-success"></span>
              </span>
              <span className="text-xs text-success font-medium">
                Connected
              </span>
            </div>
          </div>
        </div>
      </CardContent>
    </Card>
  )
}
