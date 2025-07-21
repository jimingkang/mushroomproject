import { type NextRequest, NextResponse } from "next/server"

interface Script {
  id: string
  name: string
  description: string
  path: string
  category: string
  server: string
  lastRun?: string
  status: "idle" | "running" | "scheduled"
}

// 模拟脚本数据
const scripts: Script[] = [
  {
    id: "1",
    name: "yolox_ros.sh",
    description: "Daily database backup script",
    path: "/home/a/Downloads/mushroomproject/ros2_ws/yolox_ros.sh",
    category: "Image Processing",
    server: "jetson orin nano",
    lastRun: "2025-01-17 02:00:00",
    status: "idle",
  },
  {
    id: "2",
    name: "gripper.sh",
    description: "System package updates and security patches",
    path: "/home/a/Downloads/mushroomproject/ros2_ws/gripper.sh",
    category: "Manipulation",
    server: "jetson orin nano",
    lastRun: "2025-01-17 14:15:00",
    status: "running",
  },
  {
    id: "3",
    name: "hitbot.sh",
    description: "Rotate and compress system logs",
    path: "/home/a/Downloads/mushroomproject/ros2_ws/hitbot.sh",
    category: "Manipulation",
    server: "jetson orin nano",
    lastRun: "2025-01-16 23:00:00",
    status: "scheduled",
  },
  {
    id: "4",
    name: "deploy_app.sh",
    description: "Deploy application to production",
    path: "/opt/scripts/deploy_app.sh",
    category: "Deployment",
    server: "app-server-01",
    lastRun: "2025-01-16 16:30:00",
    status: "idle",
  },
  {
    id: "5",
    name: "monitor_services.sh",
    description: "Check service health and restart if needed",
    path: "/opt/scripts/monitor_services.sh",
    category: "Monitoring",
    server: "monitor-server-01",
    lastRun: "2025-01-17 14:00:00",
    status: "idle",
  },
  {
    id: "6",
    name: "cleanup_temp.sh",
    description: "Clean temporary files and cache",
    path: "/opt/scripts/cleanup_temp.sh",
    category: "Maintenance",
    server: "web-server-02",
    lastRun: "2025-01-17 01:00:00",
    status: "idle",
  },
]

export async function GET(request: NextRequest) {
  const { searchParams } = new URL(request.url)
  const category = searchParams.get("category")
  const status = searchParams.get("status")

  let filteredScripts = scripts

  if (category && category !== "all") {
    filteredScripts = filteredScripts.filter((script) => script.category.toLowerCase() === category.toLowerCase())
  }

  if (status && status !== "all") {
    filteredScripts = filteredScripts.filter((script) => script.status === status)
  }

  return NextResponse.json({
    success: true,
    scripts: filteredScripts,
    total: filteredScripts.length,
  })
}

export async function POST(request: NextRequest) {
  try {
    const newScript = await request.json()

    // 验证必需字段
    if (!newScript.name || !newScript.path || !newScript.server) {
      return NextResponse.json({ success: false, error: "Name, path, and server are required" }, { status: 400 })
    }

    const script: Script = {
      id: (scripts.length + 1).toString(),
      name: newScript.name,
      description: newScript.description || "",
      path: newScript.path,
      category: newScript.category || "Other",
      server: newScript.server,
      status: "idle",
    }

    scripts.push(script)

    return NextResponse.json({
      success: true,
      script,
      message: "Script added successfully",
    })
  } catch (error) {
    console.error("Add script error:", error)
    return NextResponse.json({ success: false, error: "Internal server error" }, { status: 500 })
  }
}
