import Link from "next/link"
import { AlertTriangle, Battery, Clock, MapPin, MoreHorizontal, PenToolIcon as Tool, Zap } from "lucide-react"

import { Badge } from "@/components/ui/badge"
import { Button } from "@/components/ui/button"
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card"
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuLabel,
  DropdownMenuSeparator,
  DropdownMenuTrigger,
} from "@/components/ui/dropdown-menu"
import { Progress } from "@/components/ui/progress"

interface RobotCardProps {
  id: string
  name: string
  status: "active" | "maintenance" | "error" | "idle"
  type: string
  battery: number
  lastActive: string
  location: string
}

export function RobotCard({ id, name, status, type, battery, lastActive, location }: RobotCardProps) {
  return (
    <Card>
      <CardHeader className="pb-2">
        <div className="flex items-start justify-between">
          <div>
            <CardTitle>{name}</CardTitle>
            <CardDescription>{id}</CardDescription>
          </div>
          <DropdownMenu>
            <DropdownMenuTrigger asChild>
              <Button variant="ghost" size="icon">
                <MoreHorizontal className="h-4 w-4" />
                <span className="sr-only">Open menu</span>
              </Button>
            </DropdownMenuTrigger>
            <DropdownMenuContent align="end">
              <DropdownMenuLabel>Actions</DropdownMenuLabel>
              <DropdownMenuItem>View details</DropdownMenuItem>
              <DropdownMenuItem>Assign task</DropdownMenuItem>
              <DropdownMenuSeparator />
              <DropdownMenuItem>Maintenance mode</DropdownMenuItem>
              <DropdownMenuItem>Remote control</DropdownMenuItem>
              <DropdownMenuSeparator />
              <DropdownMenuItem className="text-destructive">Shutdown</DropdownMenuItem>
            </DropdownMenuContent>
          </DropdownMenu>
        </div>
        <div className="flex items-center gap-2 pt-2">
          <Badge
            variant={
              status === "active"
                ? "default"
                : status === "maintenance"
                  ? "outline"
                  : status === "error"
                    ? "destructive"
                    : "secondary"
            }
            className="capitalize"
          >
            {status === "active" && <Zap className="mr-1 h-3 w-3" />}
            {status === "maintenance" && <Tool className="mr-1 h-3 w-3" />}
            {status === "error" && <AlertTriangle className="mr-1 h-3 w-3" />}
            {status}
          </Badge>
          <Badge variant="secondary">{type}</Badge>
        </div>
      </CardHeader>
      <CardContent className="pb-2">
        <div className="flex items-center justify-between mb-2">
          <div className="flex items-center text-sm text-muted-foreground">
            <Battery className="mr-1 h-4 w-4" />
            Battery
          </div>
          <span className="text-sm font-medium">{battery}%</span>
        </div>
        <Progress value={battery} className="h-2" />
        <div className="grid grid-cols-2 gap-2 mt-4">
          <div className="flex items-center text-sm text-muted-foreground">
            <Clock className="mr-1 h-4 w-4" />
            <span>{lastActive}</span>
          </div>
          <div className="flex items-center text-sm text-muted-foreground">
            <MapPin className="mr-1 h-4 w-4" />
            <span>{location}</span>
          </div>
        </div>
      </CardContent>
      <CardFooter className="pt-2">
        <Button asChild className="w-full">
          <Link href={`/robots/${id}`}>View Details</Link>
        </Button>
      </CardFooter>
    </Card>
  )
}
