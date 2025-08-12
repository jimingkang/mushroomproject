import Link from "next/link"
import { Battery, BatteryCharging, Clock, Plus, Settings, FileText, Terminal } from "lucide-react"

import { Button } from "@/components/ui/button"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Progress } from "@/components/ui/progress"
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs"
import { RobotCard } from "@/components/robot-card"

export default function DashboardPage() {
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
          <Button size="sm" asChild>
            <Link href="/robots/new">
              <Plus className="mr-2 h-4 w-4" />
              Add Robot
            </Link>
          </Button>
        </nav>
      </header>
      <main className="flex flex-1 flex-col gap-4 p-4 md:gap-8 md:p-8">
        <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-4">
          <Card>
            <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
              <CardTitle className="text-sm font-medium">Total Robots</CardTitle>
              <BatteryCharging className="h-4 w-4 text-muted-foreground" />
            </CardHeader>
            <CardContent>
              <div className="text-2xl font-bold">12</div>
              <p className="text-xs text-muted-foreground">+2 from last month</p>
            </CardContent>
          </Card>
          <Card>
            <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
              <CardTitle className="text-sm font-medium">Active Robots</CardTitle>
              <Battery className="h-4 w-4 text-muted-foreground" />
            </CardHeader>
            <CardContent>
              <div className="text-2xl font-bold">8</div>
              <p className="text-xs text-muted-foreground">75% operational rate</p>
            </CardContent>
          </Card>
          <Card>
            <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
              <CardTitle className="text-sm font-medium">Tasks Completed</CardTitle>
              <Clock className="h-4 w-4 text-muted-foreground" />
            </CardHeader>
            <CardContent>
              <div className="text-2xl font-bold">342</div>
              <p className="text-xs text-muted-foreground">+18% from last week</p>
            </CardContent>
          </Card>
          <Card>
            <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
              <CardTitle className="text-sm font-medium">System Health</CardTitle>
              <svg
                xmlns="http://www.w3.org/2000/svg"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                strokeLinecap="round"
                strokeLinejoin="round"
                strokeWidth="2"
                className="h-4 w-4 text-muted-foreground"
              >
                <path d="M22 12h-4l-3 9L9 3l-3 9H2" />
              </svg>
            </CardHeader>
            <CardContent>
              <div className="text-2xl font-bold">98%</div>
              <Progress value={98} className="mt-2" />
            </CardContent>
          </Card>
        </div>
        <Tabs defaultValue="all">
          <div className="flex items-center">
            <TabsList>
              <TabsTrigger value="all">All Robots</TabsTrigger>
              <TabsTrigger value="active">Active</TabsTrigger>
              <TabsTrigger value="maintenance">Maintenance</TabsTrigger>
              <TabsTrigger value="error">Error</TabsTrigger>
            </TabsList>
            <div className="ml-auto flex items-center gap-2">
              <Button variant="outline" size="sm">
                Filter
              </Button>
              <Button variant="outline" size="sm">
                Sort
              </Button>
            </div>
          </div>
          <TabsContent value="all" className="mt-4 grid gap-4 md:grid-cols-2 lg:grid-cols-3">
            <RobotCard
              id="R-001"
              name="WareHouse One Bot"
              status="active"
              type="Industrial"
              battery={85}
              lastActive="2 minutes ago"
              location="Assembly Line A"
            />
            <RobotCard
              id="R-002"
              name="WareHouse Two Bot"
              status="active"
              type="Transport"
              battery={62}
              lastActive="5 minutes ago"
              location="Warehouse B"
            />
           
          </TabsContent>
          <TabsContent value="active" className="mt-4 grid gap-4 md:grid-cols-2 lg:grid-cols-3">
            <RobotCard
              id="R-001"
              name="WareHouse One Bot"
              status="active"
              type="Industrial"
              battery={85}
              lastActive="2 minutes ago"
              location="Assembly Line A"
            />
           
          </TabsContent>
          <TabsContent value="maintenance" className="mt-4 grid gap-4 md:grid-cols-2 lg:grid-cols-3">
            <RobotCard
              id="R-003"
              name="WareHouse Two Bot"
              status="maintenance"
              type="Quality Control"
              battery={24}
              lastActive="2 hours ago"
              location="Maintenance Bay"
            />
          </TabsContent>
          <TabsContent value="error" className="mt-4 grid gap-4 md:grid-cols-2 lg:grid-cols-3">
           
          </TabsContent>
        </Tabs>
      </main>
    </div>
  )
}
