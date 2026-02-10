"use client"

import { useState, useEffect } from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import { Button } from "@/components/ui/button"
import {
    Video,
    VideoOff,
    Maximize2,
    Camera,
    RefreshCw,
    Circle,
} from "lucide-react"

interface VideoFeedProps {
    cameraId: string
    name: string
    isActive?: boolean
    // Add a prop for the URL so it's reusable, but default it to your IP
    streamUrl?: string
}

export function VideoFeed({
                              cameraId,
                              name,
                              isActive = true,
                              // Default to the URL you provided
                              streamUrl = "http://172.23.248.32:5002/video_feed"
                          }: VideoFeedProps) {
    const [isStreaming, setIsStreaming] = useState(isActive)
    const [isFullscreen, setIsFullscreen] = useState(false)
    const [fps, setFps] = useState(30)
    const [latency, setLatency] = useState(45)

    // Optional: Add a timestamp refresh to keep the overlay active
    const [timestamp, setTimestamp] = useState(new Date())

    useEffect(() => {
        const interval = setInterval(() => {
            // Simulate telemetry updates (if you aren't getting these from the API)
            setFps(Math.floor(Math.random() * 5) + 28)
            setLatency(Math.floor(Math.random() * 30) + 30)
            setTimestamp(new Date())
        }, 1000)
        return () => clearInterval(interval)
    }, [])

    return (
        <Card className="border-border bg-card overflow-hidden">
            <CardHeader className="flex flex-row items-center justify-between py-3 px-4 border-b border-border">
                <div className="flex items-center gap-3">
                    <Camera className="h-4 w-4 text-muted-foreground" />
                    <CardTitle className="text-sm font-medium">{name}</CardTitle>
                    <Badge
                        variant={isStreaming ? "default" : "secondary"}
                        className={`text-xs ${isStreaming ? "bg-success/20 text-success border-success/30" : ""}`}
                    >
                        <Circle
                            className={`h-2 w-2 mr-1 ${isStreaming ? "fill-success animate-pulse" : "fill-muted-foreground"}`}
                        />
                        {isStreaming ? "Live" : "Offline"}
                    </Badge>
                </div>
                <div className="flex items-center gap-2">
                    {isStreaming && (
                        <span className="text-xs text-muted-foreground font-mono hidden sm:inline-block">
              {fps} FPS • {latency}ms
            </span>
                    )}
                    <Button
                        variant="ghost"
                        size="icon"
                        className="h-7 w-7"
                        onClick={() => setIsStreaming(!isStreaming)}
                    >
                        {isStreaming ? (
                            <VideoOff className="h-4 w-4" />
                        ) : (
                            <Video className="h-4 w-4" />
                        )}
                    </Button>
                    <Button
                        variant="ghost"
                        size="icon"
                        className="h-7 w-7"
                        onClick={() => setIsFullscreen(!isFullscreen)}
                    >
                        <Maximize2 className="h-4 w-4" />
                    </Button>
                </div>
            </CardHeader>
            <CardContent className="p-0">
                <div className="relative aspect-video bg-black">
                    {isStreaming ? (
                        <div className="absolute inset-0 flex items-center justify-center overflow-hidden">

                            {/* --- REAL VIDEO FEED --- */}
                            {/* We use a standard img tag because Next/Image doesn't handle MJPEG streams well */}
                            <img
                                src={streamUrl}
                                alt="Live Robot Feed"
                                className="w-full h-full object-contain"
                                onError={() => {
                                    // Optional: Auto-switch to offline mode if stream fails
                                    console.error("Stream failed to load");
                                    setIsStreaming(false);
                                }}
                            />

                            {/* Overlay: Timestamp */}
                            <div className="absolute bottom-2 left-2 px-2 py-1 bg-black/60 backdrop-blur-sm rounded text-xs font-mono text-white/90">
                                {timestamp.toLocaleTimeString()}
                            </div>

                            {/* Overlay: Recording Indicator */}
                            <div className="absolute top-2 right-2 flex items-center gap-1 px-2 py-1 bg-destructive/80 backdrop-blur-sm rounded">
                                <Circle className="h-2 w-2 fill-white animate-pulse" />
                                <span className="text-xs font-mono text-white font-bold">REC</span>
                            </div>

                        </div>
                    ) : (
                        <div className="absolute inset-0 flex flex-col items-center justify-center gap-3 text-muted-foreground bg-secondary/50">
                            <VideoOff className="h-12 w-12 opacity-50" />
                            <span className="text-sm">Camera offline</span>
                            <Button
                                variant="outline"
                                size="sm"
                                onClick={() => setIsStreaming(true)}
                            >
                                <RefreshCw className="h-4 w-4 mr-2" />
                                Reconnect
                            </Button>
                        </div>
                    )}
                </div>
            </CardContent>
        </Card>
    )
}