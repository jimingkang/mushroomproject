import Link from "next/link"
import { ArrowLeft, BatteryCharging, Settings, FileText, Terminal } from "lucide-react"

import { Button } from "@/components/ui/button"
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card"
import { Input } from "@/components/ui/input"
import { Label } from "@/components/ui/label"
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select"
import { Textarea } from "@/components/ui/textarea"

export default function NewRobotPage() {
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
          <h1 className="text-xl font-bold tracking-tight">Add New Robot</h1>
          <p className="text-muted-foreground">Register a new robot in the system</p>
        </div>
      </div>
      <main className="flex flex-1 flex-col gap-4 p-4 md:gap-8 md:p-8">
        <Card className="mx-auto max-w-2xl">
          <CardHeader>
            <CardTitle>Robot Information</CardTitle>
            <CardDescription>Enter the details of the new robot</CardDescription>
          </CardHeader>
          <CardContent className="space-y-4">
            <div className="grid grid-cols-2 gap-4">
              <div className="space-y-2">
                <Label htmlFor="robot-id">Robot ID</Label>
                <Input id="robot-id" placeholder="e.g., R-007" />
              </div>
              <div className="space-y-2">
                <Label htmlFor="robot-name">Robot Name</Label>
                <Input id="robot-name" placeholder="e.g., Welding Bot" />
              </div>
            </div>
            <div className="grid grid-cols-2 gap-4">
              <div className="space-y-2">
                <Label htmlFor="robot-type">Robot Type</Label>
                <Select>
                  <SelectTrigger id="robot-type">
                    <SelectValue placeholder="Select type" />
                  </SelectTrigger>
                  <SelectContent>
                    <SelectItem value="industrial">Industrial</SelectItem>
                    <SelectItem value="transport">Transport</SelectItem>
                    <SelectItem value="quality">Quality Control</SelectItem>
                    <SelectItem value="facility">Facility</SelectItem>
                    <SelectItem value="security">Security</SelectItem>
                  </SelectContent>
                </Select>
              </div>
              <div className="space-y-2">
                <Label htmlFor="robot-model">Model</Label>
                <Input id="robot-model" placeholder="e.g., RoboTech X-500" />
              </div>
            </div>
            <div className="grid grid-cols-2 gap-4">
              <div className="space-y-2">
                <Label htmlFor="serial-number">Serial Number</Label>
                <Input id="serial-number" placeholder="e.g., RT-X500-12345" />
              </div>
              <div className="space-y-2">
                <Label htmlFor="manufacture-date">Manufacture Date</Label>
                <Input id="manufacture-date" type="date" />
              </div>
            </div>
            <div className="grid grid-cols-2 gap-4">
              <div className="space-y-2">
                <Label htmlFor="location">Initial Location</Label>
                <Select>
                  <SelectTrigger id="location">
                    <SelectValue placeholder="Select location" />
                  </SelectTrigger>
                  <SelectContent>
                    <SelectItem value="assembly-a">Assembly Line A</SelectItem>
                    <SelectItem value="assembly-b">Assembly Line B</SelectItem>
                    <SelectItem value="assembly-c">Assembly Line C</SelectItem>
                    <SelectItem value="warehouse-a">Warehouse A</SelectItem>
                    <SelectItem value="warehouse-b">Warehouse B</SelectItem>
                    <SelectItem value="perimeter">Perimeter Zone</SelectItem>
                  </SelectContent>
                </Select>
              </div>
              <div className="space-y-2">
                <Label htmlFor="firmware">Firmware Version</Label>
                <Input id="firmware" placeholder="e.g., v3.2.1" />
              </div>
            </div>
            <div className="space-y-2">
              <Label htmlFor="description">Description</Label>
              <Textarea id="description" placeholder="Enter additional details about this robot..." rows={4} />
            </div>
          </CardContent>
          <CardFooter className="flex justify-between">
            <Button variant="outline" asChild>
              <Link href="/">Cancel</Link>
            </Button>
            <Button>Register Robot</Button>
          </CardFooter>
        </Card>
      </main>
    </div>
  )
}
