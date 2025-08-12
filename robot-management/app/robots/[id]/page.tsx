import Link from "next/link"
import {
  ArrowLeft,
  Battery,
  BatteryCharging,
  Clock,
  MapPin,
  Settings,
  PenToolIcon as Tool,
  Zap,
  FileText,
  Terminal,
} from "lucide-react"

import { Badge } from "@/components/ui/badge"
import { Button } from "@/components/ui/button"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Progress } from "@/components/ui/progress"
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs"
import { TaskList } from "@/components/task-list"
import { MaintenanceLog } from "@/components/maintenance-log"

export default function RobotDetailPage({ params }: { params: { id: string } }) {
  // In a real app, you would fetch the robot data based on the ID
  const robot = {
    id: params.id,
    name: "WareHouse 1 ",
    status: "active",
    type: "Industrial",
    battery: 85,
    lastActive: "2 minutes ago",
    location: "Assembly Line A",
    model: "RoboTech X-500",
    serialNumber: "RT-X500-12345",
    manufactureDate: "2024-03-15",
    lastMaintenance: "2025-06-20",
    operatingHours: 1245,
    firmware: "v3.2.1",
  }

  return (
    <div className="flex min-h-screen w-full flex-col">
      <header className="sticky top-0 z-10 flex h-16 items-center gap-4 border-b bg-background px-4 md:px-6">
        <Link href="/" className="flex items-center gap-2 font-semibold">
          <BatteryCharging className="h-6 w-6" />
          <span>RoboManager</span>
        </Link>
        <nav className="ml-auto flex gap-2">
          <Button variant="outline" size="sm" asChild>
            <Link href="/logs">
              <FileText className="mr-2 h-4 w-4" />
              Logs
            </Link>
          </Button>
          <Button variant="outline" size="sm" asChild>
            <Link href="/scripts">
              <Terminal className="mr-2 h-4 w-4" />
              Scripts
            </Link>
          </Button>
          <Button variant="outline" size="sm" asChild>
            <Link href="/settings">
              <Settings className="mr-2 h-4 w-4" />
              Settings
            </Link>
          </Button>
        </nav>
      </header>
      <div className="flex items-center gap-4 border-b bg-background px-4 py-4 md:px-6">
        <Button variant="ghost" size="icon" asChild>
          <Link href="/">
            <ArrowLeft className="h-4 w-4" />
            <span className="sr-only">Back</span>
          </Link>
        </Button>
        <div>
          <h1 className="text-xl font-bold tracking-tight">
            {robot.name} <span className="text-muted-foreground">({robot.id})</span>
          </h1>
          <div className="flex items-center gap-2">
            <Badge
              variant={
                robot.status === "active"
                  ? "default"
                  : robot.status === "maintenance"
                    ? "outline"
                    : robot.status === "error"
                      ? "destructive"
                      : "secondary"
              }
              className="capitalize"
            >
              {robot.status === "active" && <Zap className="mr-1 h-3 w-3" />}
              {robot.status === "maintenance" && <Tool className="mr-1 h-3 w-3" />}
              {robot.status}
            </Badge>
            <Badge variant="secondary">{robot.type}</Badge>
          </div>
        </div>
        <div className="ml-auto flex gap-2">
          <Button variant="outline">Remote Control</Button>
          <Button variant="outline">Maintenance</Button>
          <Button>Assign Task</Button>
        </div>
      </div>
      <main className="flex flex-1 flex-col gap-4 p-4 md:gap-8 md:p-8">
        <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-3">
          <Card>
            <CardHeader className="pb-2">
              <CardTitle className="text-base">Battery Status</CardTitle>
            </CardHeader>
            <CardContent>
              <div className="flex items-center justify-between mb-2">
                <div className="flex items-center text-sm">
                  <Battery className="mr-1 h-4 w-4" />
                  Current Level
                </div>
                <span className="text-lg font-bold">{robot.battery}%</span>
              </div>
              <Progress value={robot.battery} className="h-2 mb-4" />
              <div className="grid grid-cols-2 gap-2 text-sm">
                <div className="flex items-center">
                  <Clock className="mr-1 h-4 w-4 text-muted-foreground" />
                  <span className="text-muted-foreground">Est. Runtime:</span>
                </div>
                <div className="text-right font-medium">8.5 hours</div>
                <div className="flex items-center">
                  <Zap className="mr-1 h-4 w-4 text-muted-foreground" />
                  <span className="text-muted-foreground">Charge Cycles:</span>
                </div>
                <div className="text-right font-medium">124</div>
              </div>
            </CardContent>
          </Card>
          <Card>
            <CardHeader className="pb-2">
              <CardTitle className="text-base">Location</CardTitle>
            </CardHeader>
            <CardContent>
              <div className="flex items-center justify-between mb-4">
                <div className="flex items-center text-sm">
                  <MapPin className="mr-1 h-4 w-4" />
                  Current Location
                </div>
                <span className="text-lg font-bold">{robot.location}</span>
              </div>
              <div className="aspect-video bg-muted rounded-md flex items-center justify-center">
                <p className="text-sm text-muted-foreground">Location Map</p>
              </div>
            </CardContent>
          </Card>
          <Card>
            <CardHeader className="pb-2">
              <CardTitle className="text-base">Specifications</CardTitle>
            </CardHeader>
            <CardContent>
              <div className="grid grid-cols-2 gap-2 text-sm">
                <div className="text-muted-foreground">Model:</div>
                <div className="font-medium">{robot.model}</div>
                <div className="text-muted-foreground">Serial Number:</div>
                <div className="font-medium">{robot.serialNumber}</div>
                <div className="text-muted-foreground">Manufacture Date:</div>
                <div className="font-medium">{robot.manufactureDate}</div>
                <div className="text-muted-foreground">Last Maintenance:</div>
                <div className="font-medium">{robot.lastMaintenance}</div>
                <div className="text-muted-foreground">Operating Hours:</div>
                <div className="font-medium">{robot.operatingHours}</div>
                <div className="text-muted-foreground">Firmware:</div>
                <div className="font-medium">{robot.firmware}</div>
              </div>
            </CardContent>
          </Card>
        </div>
        <Tabs defaultValue="tasks">
          <TabsList>
            <TabsTrigger value="tasks">Tasks</TabsTrigger>
            <TabsTrigger value="maintenance">Maintenance Log</TabsTrigger>
            <TabsTrigger value="performance">Performance</TabsTrigger>
          </TabsList>
          <TabsContent value="tasks" className="mt-4">
            <TaskList robotId={params.id} />
          </TabsContent>
          <TabsContent value="maintenance" className="mt-4">
            <MaintenanceLog robotId={params.id} />
          </TabsContent>
          <TabsContent value="performance" className="mt-4">
            <Card>
              <CardHeader>
                <CardTitle>Performance Metrics</CardTitle>
                <CardDescription>Robot performance over the last 30 days</CardDescription>
              </CardHeader>
              <CardContent>
                <div className="h-[300px] flex items-center justify-center bg-muted rounded-md">
                  <p className="text-muted-foreground">Performance chart would be displayed here</p>
                </div>
              </CardContent>
            </Card>
          </TabsContent>
        </Tabs>
      </main>
    </div>
  )
}
