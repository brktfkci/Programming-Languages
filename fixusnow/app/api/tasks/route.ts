import { NextRequest, NextResponse } from "next/server";
import { auth } from "@/lib/auth";
import { prisma } from "@/lib/prisma";
import { z } from "zod";

const createSchema = z.object({
  title: z.string().min(5),
  description: z.string().min(10),
  category: z.string(),
  budget: z.number().optional(),
  address: z.string().min(5),
  scheduledAt: z.string().optional(),
});

export async function GET(req: NextRequest) {
  const { searchParams } = new URL(req.url);
  const category = searchParams.get("category");
  const status = searchParams.get("status") || "OPEN";

  const tasks = await prisma.task.findMany({
    where: {
      ...(category ? { category } : {}),
      status: status as "OPEN" | "IN_PROGRESS" | "COMPLETED" | "CANCELLED",
    },
    include: {
      customer: { select: { id: true, name: true, image: true } },
      applications: { select: { id: true } },
    },
    orderBy: { createdAt: "desc" },
  });

  return NextResponse.json(tasks);
}

export async function POST(req: NextRequest) {
  const session = await auth();
  if (!session?.user) {
    return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
  }

  const user = session.user as { id: string; role: string };
  if (user.role !== "CUSTOMER") {
    return NextResponse.json({ error: "Only customers can post tasks" }, { status: 403 });
  }

  const body = await req.json();
  const parsed = createSchema.safeParse(body);
  if (!parsed.success) {
    return NextResponse.json({ error: "Invalid input" }, { status: 400 });
  }

  const task = await prisma.task.create({
    data: {
      ...parsed.data,
      scheduledAt: parsed.data.scheduledAt ? new Date(parsed.data.scheduledAt) : undefined,
      customerId: user.id,
    },
  });

  return NextResponse.json(task, { status: 201 });
}
