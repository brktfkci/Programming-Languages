import { NextRequest, NextResponse } from "next/server";
import { auth } from "@/lib/auth";
import { prisma } from "@/lib/prisma";
import { z } from "zod";

const schema = z.object({
  taskId: z.string(),
  price: z.number().positive(),
  message: z.string().optional(),
});

export async function POST(req: NextRequest) {
  const session = await auth();
  if (!session?.user) return NextResponse.json({ error: "Unauthorized" }, { status: 401 });

  const user = session.user as { id: string; role: string };
  if (user.role !== "PROVIDER") {
    return NextResponse.json({ error: "Only providers can apply" }, { status: 403 });
  }

  const body = await req.json();
  const parsed = schema.safeParse(body);
  if (!parsed.success) return NextResponse.json({ error: "Invalid input" }, { status: 400 });

  const existing = await prisma.application.findUnique({
    where: { taskId_providerId: { taskId: parsed.data.taskId, providerId: user.id } },
  });
  if (existing) return NextResponse.json({ error: "Already applied" }, { status: 409 });

  const app = await prisma.application.create({
    data: { ...parsed.data, providerId: user.id },
  });

  return NextResponse.json(app, { status: 201 });
}
